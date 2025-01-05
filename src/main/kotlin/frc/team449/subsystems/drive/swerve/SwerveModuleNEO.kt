package frc.team449.subsystems.drive.swerve

import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import frc.team449.system.encoder.AbsoluteEncoder.Companion.createAbsoluteEncoder
import frc.team449.system.encoder.Encoder
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.createSparkMax
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sign

/**
 * Controls a Swerve Module.
 * @param name The name of the module (used for logging).
 * @param drivingMotor The motor that controls the speed of the module.
 * @param turningMotor The motor that controls the angle of the module
 * @param driveController The velocity control for speed of the module
 * @param turnController The position control for the angle of the module
 * @param driveFeedforward The voltage predicting equation for a given speed of the module.
 * @param location The location of the module in reference to the center of the robot.
 * NOTE: In relation to the robot [+X is forward, +Y is left, and +THETA is Counter Clock-Wise].
 */
open class SwerveModuleNEO(
  private val name: String,
  private val drivingMotor: SparkMax,
  private val turningMotor: SparkMax,
  private val turnEncoder: Encoder,
  private val driveController: PIDController,
  private val turnController: PIDController,
  private val driveFeedforward: SimpleMotorFeedforward,
  override val location: Translation2d
) : SwerveModule {
  init {
    turnController.enableContinuousInput(.0, 2 * PI)
    driveController.reset()
    turnController.reset()
  }

  override val desiredState = SwerveModuleState(
    0.0,
    Rotation2d()
  )

  /** The module's [SwerveModuleState], containing speed and angle. */
  override var state: SwerveModuleState
    get() {
      return SwerveModuleState(
        drivingMotor.encoder.velocity,
        Rotation2d(turnEncoder.position)
      )
    }
    set(desState) {
      if (abs(desState.speedMetersPerSecond) < .001) {
        stop()
        return
      }
      /** Ensure the module doesn't turn more than 90 degrees. */
      desState.optimize(Rotation2d(turnEncoder.position))

      turnController.setpoint = desState.angle.radians
      driveController.setpoint = desState.speedMetersPerSecond
      desiredState.speedMetersPerSecond = desState.speedMetersPerSecond
      desiredState.angle = desState.angle
    }

  /** The module's [SwerveModulePosition], containing distance and angle. */
  override val position: SwerveModulePosition
    get() {
      return SwerveModulePosition(
        drivingMotor.encoder.position,
        Rotation2d(turnEncoder.position)
      )
    }

  override fun setVoltage(volts: Double) {
    driveController.setpoint = 0.0
    desiredState.speedMetersPerSecond = 0.0
    turnController.setpoint = 0.0

    turningMotor.set(turnController.calculate(turnEncoder.position))
    drivingMotor.setVoltage(volts)
  }

  /** Set module speed to zero but keep module angle the same. */
  override fun stop() {
    turnController.setpoint = turnEncoder.position
    desiredState.speedMetersPerSecond = 0.0
  }

  override fun update() {
    /** CONTROL speed of module */
    val drivePid = driveController.calculate(
      drivingMotor.encoder.velocity
    )
    val driveFF = driveFeedforward.calculate(
      MetersPerSecond.of(desiredState.speedMetersPerSecond)
    )
    // .calculate(

    drivingMotor.setVoltage(drivePid + driveFF.`in`(Volts))

    /** CONTROL direction of module */
    val turnPid = turnController.calculate(
      turningMotor.encoder.position
    )

    turningMotor.set(
      turnPid +
        sign(desiredState.angle.radians - turnEncoder.position) *
          SwerveConstants.STEER_KS
    )
  }

  companion object {
    /** Create a real or simulated [SwerveModule] based on the simulation status of the robot. */
    fun createNEOModule(
      name: String,
      driveID: Int,
      driveInverted: Boolean,
      turnID: Int,
      turnInverted: Boolean,
      turnEncoderChannel: Int,
      turnEncoderOffset: Double,
      turnEncoderInverted: Boolean,
      location: Translation2d
    ): SwerveModule {
      val driveMotor = createSparkMax(
        driveID,
        driveInverted,
        true,
        gearing = SwerveConstants.DRIVE_GEARING,
        upr = SwerveConstants.DRIVE_UPR,
        currentLimit = SwerveConstants.DRIVE_SUPPLY_LIMIT
      )
      val turnMotor = createSparkMax(
        turnID,
        turnInverted,
        false,
        gearing = SwerveConstants.DRIVE_GEARING,
        upr = SwerveConstants.DRIVE_UPR,
        currentLimit = SwerveConstants.STEERING_CURRENT_LIM
      )
      val turnEncoder = createAbsoluteEncoder(
        "$name Turn Encoder",
        turnEncoderChannel,
        turnEncoderOffset,
        SwerveConstants.TURN_UPR,
        turnEncoderInverted
      )
      val driveController = PIDController(
        SwerveConstants.DRIVE_KP,
        SwerveConstants.DRIVE_KI,
        SwerveConstants.DRIVE_KD
      )
      val turnController = PIDController(
        SwerveConstants.TURN_KP,
        SwerveConstants.TURN_KI,
        SwerveConstants.TURN_KD
      )
      val driveFeedforward = SimpleMotorFeedforward(
        SwerveConstants.DRIVE_KS,
        SwerveConstants.DRIVE_KV,
        SwerveConstants.DRIVE_KA
      )
      if (RobotBase.isReal()) {
        return SwerveModuleNEO(
          name,
          driveMotor,
          turnMotor,
          turnEncoder,
          driveController,
          turnController,
          driveFeedforward,
          location
        )
      } else {
        return SwerveModuleSimNEO(
          name,
          driveMotor,
          turnMotor,
          turnEncoder,
          driveController,
          turnController,
          driveFeedforward,
          location
        )
      }
    }
  }
}

/** A "simulated" swerve module. Immediately reaches to its desired state. */
class SwerveModuleSimNEO(
  name: String,
  drivingMotor: SparkMax,
  turningMotor: SparkMax,
  turnEncoder: Encoder,
  driveController: PIDController,
  turnController: PIDController,
  driveFeedforward: SimpleMotorFeedforward,
  location: Translation2d
) : SwerveModuleNEO(
  name,
  drivingMotor,
  turningMotor,
  turnEncoder,
  driveController,
  turnController,
  driveFeedforward,
  location
) {
  private val turningMotorEncoder = Encoder.SimController(turnEncoder)
  private val driveEncoder = Encoder.SimController(
    NEOEncoder.creator(
      SwerveConstants.DRIVE_UPR,
      SwerveConstants.DRIVE_GEARING
    ).create(
      "drive encoder",
      drivingMotor,
      true
    )
  )
  private var prevTime = Timer.getFPGATimestamp()
  override var state: SwerveModuleState
    get() = SwerveModuleState(
      driveEncoder.velocity,
      Rotation2d(turningMotorEncoder.position)
    )
    set(desiredState) {
      super.state = desiredState
      turningMotorEncoder.position = desiredState.angle.radians
      driveEncoder.velocity = desiredState.speedMetersPerSecond
    }
  override val position: SwerveModulePosition
    get() = SwerveModulePosition(
      driveEncoder.position,
      Rotation2d(turningMotorEncoder.position)
    )

  override fun update() {
    val currTime = Timer.getFPGATimestamp()
    driveEncoder.position += driveEncoder.velocity * (currTime - prevTime)
    prevTime = currTime
  }
}
