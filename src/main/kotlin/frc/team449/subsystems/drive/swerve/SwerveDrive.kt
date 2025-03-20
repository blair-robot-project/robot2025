package frc.team449.subsystems.drive.swerve

import choreo.trajectory.SwerveSample
import dev.doglog.DogLog
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Robot
import frc.team449.auto.AutoConstants
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveModuleKraken.Companion.createKrakenModule
import frc.team449.subsystems.drive.swerve.SwerveModuleNEO.Companion.createNEOModule
import kotlin.math.hypot

/**
 * A Swerve Drive chassis.
 * @param modules An array of [SwerveModule]s that are on the drivetrain.
 * @param maxLinearSpeed The maximum translation speed of the chassis.
 * @param maxRotSpeed The maximum rotation speed of the chassis.
 * @param field The SmartDashboard [Field2d] widget that shows the robot's pose.
 */
open class SwerveDrive(
  val modules: List<SwerveModule>,
  var maxLinearSpeed: Double,
  var accel: Double,
  var maxRotSpeed: Double,
  protected val field: Field2d,
  val maxModuleSpeed: Double
) : SubsystemBase() {

  var pose: Pose2d = Pose2d()

  /** The kinematics that convert [ChassisSpeeds] into multiple [SwerveModuleState] objects. */
  val kinematics = SwerveDriveKinematics(
    *this.modules.map { it.location }.toTypedArray()
  )

  /** The current speed of the robot's drive. */
  var currentSpeeds = ChassisSpeeds()

  var desiredSpeeds: ChassisSpeeds = ChassisSpeeds()

  var desiredAngle = 0.0
  var desiredOmega = 0.0

  // Removed magic numbers
  private val xController: PIDController
    get() = PIDController(AutoConstants.DEFAULT_X_KP, 0.0, 0.0)
  private val yController: PIDController
    get() = PIDController(AutoConstants.DEFAULT_Y_KP, 0.0, 0.0)
  private val headingController: PIDController
    get() = PIDController(AutoConstants.DEFAULT_ROTATION_KP, 0.0, 0.0)

  init {
    headingController.enableContinuousInput(-Math.PI, Math.PI)
  }
  fun followTrajectory(
    robot: Robot,
    sample: SwerveSample
  ) {
    desiredAngle = MathUtil.angleModulus(sample.heading)
    desiredOmega = sample.omega
    val speeds = ChassisSpeeds(
      sample.vx + xController.calculate(robot.poseSubsystem.pose.x, sample.x),
      sample.vy + yController.calculate(robot.poseSubsystem.pose.y, sample.y),
      sample.omega + headingController.calculate(
        robot.poseSubsystem.pose.rotation.minus(Rotation2d.fromRadians(MathUtil.angleModulus(sample.heading))).radians
      )
    )
    val newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      speeds,
      robot.poseSubsystem.heading
    )

    // Apply the generated speeds
    set(newSpeeds)
  }

  fun set(desiredSpeeds: ChassisSpeeds) {
    this.desiredSpeeds = desiredSpeeds
    // Converts the desired [ChassisSpeeds] into an array of [SwerveModuleState].
    val desiredModuleStates =
      this.kinematics.toSwerveModuleStates(this.desiredSpeeds)
    // Scale down module speed if a module is going faster than the max speed, and prevent early desaturation.
//    normalizeDrive(desiredModuleStates, desiredSpeeds)
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredModuleStates,
      maxModuleSpeed
    )
    for (i in this.modules.indices) {
      this.modules[i].state = desiredModuleStates[i]
    }

    for (module in modules)
      module.update()
  }

  fun setVoltage(volts: Double) {
    modules.forEach {
      it.setVoltage(volts)
    }
  }

  fun getModuleVel(): Double {
    var totalVel = 0.0
    modules.forEach { totalVel += it.state.speedMetersPerSecond }
    return totalVel / modules.size
  }

  /** Stops the robot's drive. */
  fun driveStop(): Command {
    return runOnce {
      set(ChassisSpeeds(0.0, 0.0, 0.0))
    }
  }

  override fun periodic() {
    // Updates the robot's currentSpeeds.
    currentSpeeds = kinematics.toChassisSpeeds(
      arrayOf(
        modules[0].state,
        modules[1].state,
        modules[2].state,
        modules[3].state
      )
    )

    logData()
  }

  /** Stops the robot's drive. */
  fun stop() {
    this.set(ChassisSpeeds(0.0, 0.0, 0.0))
  }

  /** @return An array of [SwerveModulePosition] for each module, containing distance and angle. */
  fun getPositions(): Array<SwerveModulePosition> {
    return Array(modules.size) { i -> modules[i].position }
  }

  /** @return An array of [SwerveModuleState] for each module, containing speed and angle. */
  private fun getStates(): Array<SwerveModuleState> {
    return Array(modules.size) { i -> modules[i].state }
  }

  fun logData() {
    DogLog.log("Swerve/FL Module Current State", modules[0].state)
    DogLog.log("Swerve/FR Module Current State", modules[1].state)
    DogLog.log("Swerve/BL Module Current State", modules[2].state)
    DogLog.log("Swerve/BR Module Current State", modules[3].state)

    DogLog.log("Swerve/FL Module Desired State", modules[0].desiredState)
    DogLog.log("Swerve/FR Module Desired State", modules[1].desiredState)
    DogLog.log("Swerve/BL Module Desired State", modules[2].desiredState)
    DogLog.log("Swerve/BR Module Desired State", modules[3].desiredState)

    DogLog.log("Swerve/Current Speeds", currentSpeeds)
    DogLog.log("Swerve/Desired Speeds", desiredSpeeds)
    DogLog.log("Swerve/Translation Speed", hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond))
  }

  companion object {
    /** Create a [SwerveDrive] using [SwerveConstants]. */
    fun createSwerveKraken(field: Field2d): SwerveDrive {
      val modules = listOf(
        createKrakenModule(
          "FLModule",
          SwerveConstants.DRIVE_MOTOR_FL,
          SwerveConstants.DRIVE_INVERTED,
          SwerveConstants.TURN_MOTOR_FL,
          SwerveConstants.TURN_INVERTED,
          SwerveConstants.TURN_ENC_CHAN_FL,
          SwerveConstants.TURN_ENC_OFFSET_FL,
          SwerveConstants.TURN_ENC_INVERTED,
          Translation2d(
            SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
            SwerveConstants.TRACKWIDTH / 2
          )
        ),
        createKrakenModule(
          "FRModule",
          SwerveConstants.DRIVE_MOTOR_FR,
          SwerveConstants.DRIVE_INVERTED,
          SwerveConstants.TURN_MOTOR_FR,
          SwerveConstants.TURN_INVERTED,
          SwerveConstants.TURN_ENC_CHAN_FR,
          SwerveConstants.TURN_ENC_OFFSET_FR,
          SwerveConstants.TURN_ENC_INVERTED,
          Translation2d(
            SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
            -SwerveConstants.TRACKWIDTH / 2
          )
        ),
        createKrakenModule(
          "BLModule",
          SwerveConstants.DRIVE_MOTOR_BL,
          SwerveConstants.DRIVE_INVERTED,
          SwerveConstants.TURN_MOTOR_BL,
          SwerveConstants.TURN_INVERTED,
          SwerveConstants.TURN_ENC_CHAN_BL,
          SwerveConstants.TURN_ENC_OFFSET_BL,
          SwerveConstants.TURN_ENC_INVERTED,
          Translation2d(
            -SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
            SwerveConstants.TRACKWIDTH / 2
          )
        ),
        createKrakenModule(
          "BLModule",
          SwerveConstants.DRIVE_MOTOR_BR,
          SwerveConstants.DRIVE_INVERTED,
          SwerveConstants.TURN_MOTOR_BR,
          SwerveConstants.TURN_INVERTED,
          SwerveConstants.TURN_ENC_CHAN_BR,
          SwerveConstants.TURN_ENC_OFFSET_BR,
          SwerveConstants.TURN_ENC_INVERTED,
          Translation2d(
            -SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
            -SwerveConstants.TRACKWIDTH / 2
          )
        )
      )
      return if (isReal()) {
        SwerveDrive(
          modules,
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ACCEL,
          RobotConstants.MAX_ROT_SPEED,
          field,
          SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED
        )
      } else {
        SwerveSim(
          modules,
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ACCEL,
          RobotConstants.MAX_ROT_SPEED,
          field,
          SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED
        )
      }
    }

    fun createSwerveNEO(field: Field2d): SwerveDrive {
      val modules = listOf(
        createNEOModule(
          "FLModule",
          SwerveConstants.DRIVE_MOTOR_FL,
          SwerveConstants.DRIVE_INVERTED,
          SwerveConstants.TURN_MOTOR_FL,
          SwerveConstants.TURN_INVERTED,
          SwerveConstants.TURN_ENC_CHAN_FL,
          SwerveConstants.TURN_ENC_OFFSET_FL,
          SwerveConstants.TURN_ENC_INVERTED,
          Translation2d(
            SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
            SwerveConstants.TRACKWIDTH / 2
          )
        ),
        createNEOModule(
          "FRModule",
          SwerveConstants.DRIVE_MOTOR_FR,
          SwerveConstants.DRIVE_INVERTED,
          SwerveConstants.TURN_MOTOR_FR,
          SwerveConstants.TURN_INVERTED,
          SwerveConstants.TURN_ENC_CHAN_FR,
          SwerveConstants.TURN_ENC_OFFSET_FR,
          SwerveConstants.TURN_ENC_INVERTED,
          Translation2d(
            SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
            -SwerveConstants.TRACKWIDTH / 2
          )
        ),
        createNEOModule(
          "BLModule",
          SwerveConstants.DRIVE_MOTOR_BL,
          SwerveConstants.DRIVE_INVERTED,
          SwerveConstants.TURN_MOTOR_BL,
          SwerveConstants.TURN_INVERTED,
          SwerveConstants.TURN_ENC_CHAN_BL,
          SwerveConstants.TURN_ENC_OFFSET_BL,
          SwerveConstants.TURN_ENC_INVERTED,
          Translation2d(
            -SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
            SwerveConstants.TRACKWIDTH / 2
          )
        ),
        createNEOModule(
          "BLModule",
          SwerveConstants.DRIVE_MOTOR_BR,
          SwerveConstants.DRIVE_INVERTED,
          SwerveConstants.TURN_MOTOR_BR,
          SwerveConstants.TURN_INVERTED,
          SwerveConstants.TURN_ENC_CHAN_BR,
          SwerveConstants.TURN_ENC_OFFSET_BR,
          SwerveConstants.TURN_ENC_INVERTED,
          Translation2d(
            -SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
            -SwerveConstants.TRACKWIDTH / 2
          )
        )
      )
      return if (isReal()) {
        SwerveDrive(
          modules,
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ACCEL,
          RobotConstants.MAX_ROT_SPEED,
          field,
          SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED
        )
      } else {
        SwerveSim(
          modules,
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ACCEL,
          RobotConstants.MAX_ROT_SPEED,
          field,
          SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED
        )
      }
    }
  }
}
