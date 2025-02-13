package frc.team449.subsystems.drive.swerve
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.RobotBase.isReal
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveModuleKraken.Companion.createKrakenModule
import frc.team449.subsystems.drive.swerve.SwerveModuleNEO.Companion.createNEOModule
import frc.team449.subsystems.drive.swerve.sim.SwerveModuleSim.Companion.createModuleSim
import org.ironmaple.simulation.drivesims.COTS
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig
import kotlin.math.hypot

/**
 * A Swerve Drive chassis.
 * @param modules An array of [SwerveModule]s that are on the drivetrain.
 * @param ahrs The gyro that is mounted on the chassis.
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

  /** The kinematics that convert [ChassisSpeeds] into multiple [SwerveModuleState] objects. */
  val kinematics = SwerveDriveKinematics(
    *this.modules.map { it.location }.toTypedArray()
  )

  /** The current speed of the robot's drive. */
  var currentSpeeds = ChassisSpeeds()

  var desiredSpeeds: ChassisSpeeds = ChassisSpeeds()

  protected var speedMagnitude: Double = 0.0

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

    speedMagnitude = hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
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

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("3.0", "Driving & Steering (Std Order FL, FR, BL, BR)")
    builder.addDoubleArrayProperty(
      "3.1 Current States",
      {
        doubleArrayOf(
          modules[0].state.angle.radians,
          modules[0].state.speedMetersPerSecond,
          modules[1].state.angle.radians,
          modules[1].state.speedMetersPerSecond,
          modules[2].state.angle.radians,
          modules[2].state.speedMetersPerSecond,
          modules[3].state.angle.radians,
          modules[3].state.speedMetersPerSecond,
        )
      },
      null
    )
    builder.addDoubleArrayProperty(
      "3.2 Desired States",
      {
        doubleArrayOf(
          modules[0].desiredState.angle.radians,
          modules[0].desiredState.speedMetersPerSecond,
          modules[1].desiredState.angle.radians,
          modules[1].desiredState.speedMetersPerSecond,
          modules[2].desiredState.angle.radians,
          modules[2].desiredState.speedMetersPerSecond,
          modules[3].desiredState.angle.radians,
          modules[3].desiredState.speedMetersPerSecond,
        )
      },
      null
    )

    builder.addDoubleArrayProperty(
      "3.3 Steering Rotation",
      {
        doubleArrayOf(
          modules[0].state.angle.rotations,
          modules[1].state.angle.rotations,
          modules[2].state.angle.rotations,
          modules[3].state.angle.rotations,
        )
      },
      null
    )
  }

  companion object {
    /** Create a [SwerveDrive] using [SwerveConstants]. */
    fun createSwerveKraken(field: Field2d): SwerveDrive {
      val modules: List<SwerveModule>;
      if (isReal()) {
        modules = listOf(
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
      }
      else {
        val swerveDriveMapleSim: SwerveDriveSimulation = SwerveDriveSimulation(
          DriveTrainSimulationConfig(
            SwerveConstants.ROBOTMASS,
            SwerveConstants.BUMPERLENGTH_X,
            SwerveConstants.BUMPERLENGTH_Y,
            Meters.of(SwerveConstants.WHEELBASE),
            Meters.of(SwerveConstants.TRACKWIDTH),
            SwerveModuleSimulationConfig(
              DCMotor.getKrakenX60(1),
              DCMotor.getKrakenX60(1),
              SwerveConstants.DRIVE_GEARING,
              SwerveConstants.TURN_GEARING,
              SwerveConstants.DRIVE_FRICTION_VOLTAGE,
              SwerveConstants.TURN_FRICTION_VOLTAGE,
              Meters.of(SwerveConstants.WHEEL_RADIUS),
              SwerveConstants.STEER_ROTATIONAL_INERTIA,
              SwerveConstants.WHEEL_COEFFICIENT_OF_FRICTION
            ),
            COTS.ofNav2X()
          ),
          Pose2d()
        );
        modules = listOf(
          createModuleSim(
            "FLModule",
            swerveDriveMapleSim.modules[0],
            Translation2d(
              SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
              SwerveConstants.TRACKWIDTH / 2
            )
          ),
          createModuleSim(
            "FRModule",
            swerveDriveMapleSim.modules[1],
            Translation2d(
              SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
              -SwerveConstants.TRACKWIDTH / 2
            )
          ),
          createModuleSim(
            "BLModule",
            swerveDriveMapleSim.modules[2],
            Translation2d(
              -SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
              SwerveConstants.TRACKWIDTH / 2
            )
          ),
          createModuleSim(
            "BLModule",
            swerveDriveMapleSim.modules[3],
            Translation2d(
              -SwerveConstants.WHEELBASE / 2 - SwerveConstants.X_SHIFT,
              -SwerveConstants.TRACKWIDTH / 2
            )
          )
        );
      }
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
