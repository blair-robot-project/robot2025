package frc.team449.subsystems.drive.swerve

import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import frc.team449.system.encoder.AbsoluteEncoder
import frc.team449.system.encoder.Encoder
import frc.team449.system.motor.createKraken
import frc.team449.system.motor.createSparkMax
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sign

/**
 * Controls a Swerve Module.
 *
 * @param name The name of the module (used for logging).
 * @param drivingMotor The motor that controls the speed of the module.
 * @param turningMotor The motor that controls the angle of the module
 * @param turnController The position control for the angle of the module
 * @param location The location of the module in reference to the center of the robot. NOTE: In
 *   relation to the robot [+X is forward, +Y is left, and +THETA is Counter Clock-Wise].
 */
open class SwerveModuleKraken(
    private val name: String,
    private val drivingMotor: TalonFX,
    private val turningMotor: SparkMax,
    private val turnEncoder: Encoder,
    val turnController: PIDController,
    override val location: Translation2d,
) : SwerveModule {
    init {
        turnController.enableContinuousInput(.0, 2 * PI)
        turnController.reset()
    }

    override val desiredState = SwerveModuleState(0.0, Rotation2d())

    /** The module's [SwerveModuleState], containing speed and angle. */
    override var state: SwerveModuleState
        get() {
            return SwerveModuleState(
                drivingMotor.velocity.valueAsDouble,
                Rotation2d(turnEncoder.position),
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
            desiredState.speedMetersPerSecond = desState.speedMetersPerSecond
            desiredState.angle = desState.angle
        }

    /** The module's [SwerveModulePosition], containing distance and angle. */
    override val position: SwerveModulePosition
        get() {
            return SwerveModulePosition(
                drivingMotor.position.value.magnitude(),
                Rotation2d(turnEncoder.position),
            )
        }

    override fun setVoltage(volts: Double) {
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
        drivingMotor.setControl(
            VelocityVoltage(desiredState.speedMetersPerSecond)
                .withUpdateFreqHz(SwerveConstants.KRAKEN_UPDATE_RATE)
                .withEnableFOC(SwerveConstants.USE_FOC)
        )

        /** CONTROL direction of module */
        val turnPid = turnController.calculate(turnEncoder.position)

        turningMotor.set(
            turnPid +
                sign(desiredState.angle.radians - turnEncoder.position) * SwerveConstants.STEER_KS
        )
    }

    companion object {
        /**
         * Create a real or simulated [SwerveModule] based on the simulation status of the robot.
         */
        fun createKrakenModule(
            name: String,
            driveID: Int,
            driveInverted: Boolean,
            turnID: Int,
            turnInverted: Boolean,
            turnEncoderChannel: Int,
            turnEncoderOffset: Double,
            turnEncoderInverted: Boolean,
            location: Translation2d,
        ): SwerveModuleKraken {
            val drivingMotor =
                createKraken(
                    driveID,
                    driveInverted,
                    true,
                    1 / (SwerveConstants.DRIVE_GEARING * SwerveConstants.DRIVE_UPR),
                    SwerveConstants.DUTY_CYCLE_DEADBAND,
                    SwerveConstants.DRIVE_STATOR_LIMIT,
                    SwerveConstants.DRIVE_SUPPLY_BOOST,
                    SwerveConstants.DRIVE_SUPPLY_BOOST_TIME,
                    SwerveConstants.DRIVE_SUPPLY_LIMIT,
                    SwerveConstants.DRIVE_FOC_CURRENT_LIMIT,
                    SwerveConstants.DRIVE_KS,
                    SwerveConstants.DRIVE_KV,
                    SwerveConstants.DRIVE_KA,
                    SwerveConstants.DRIVE_KP,
                    SwerveConstants.DRIVE_KI,
                    SwerveConstants.DRIVE_KD,
                    updateFrequency = SwerveConstants.VALUE_UPDATE_RATE,
                )
            val turnMotor =
                createSparkMax(
                    turnID,
                    turnInverted,
                    false,
                    gearing = SwerveConstants.DRIVE_GEARING,
                    upr = SwerveConstants.DRIVE_UPR,
                    currentLimit = SwerveConstants.STEERING_CURRENT_LIM,
                )
            val turnEncoder =
                AbsoluteEncoder.createAbsoluteEncoder(
                    "$name Turn Encoder",
                    turnEncoderChannel,
                    turnEncoderOffset,
                    SwerveConstants.TURN_UPR,
                    turnEncoderInverted,
                )
            val turnController =
                PIDController(
                    SwerveConstants.TURN_KP,
                    SwerveConstants.TURN_KI,
                    SwerveConstants.TURN_KD,
                )
            if (RobotBase.isReal()) {
                return SwerveModuleKraken(
                    name,
                    drivingMotor,
                    turnMotor,
                    turnEncoder,
                    turnController,
                    location,
                )
            } else {
                return SwerveModuleSimKraken(
                    name,
                    drivingMotor,
                    turnMotor,
                    turnEncoder,
                    turnController,
                    location,
                )
            }
        }
    }
}

/** A "simulated" swerve module. Immediately reaches to its desired state. */
class SwerveModuleSimKraken(
    name: String,
    drivingMotor: TalonFX,
    turningMotor: SparkMax,
    turnEncoder: Encoder,
    turnController: PIDController,
    location: Translation2d,
) : SwerveModuleKraken(name, drivingMotor, turningMotor, turnEncoder, turnController, location) {
    private val turningMotorEncoder = Encoder.SimController(turnEncoder)

    private var drivePosition = 0.0
    private var driveVelocity = 0.0

    private var prevTime = Timer.getFPGATimestamp()
    override var state: SwerveModuleState
        get() = SwerveModuleState(driveVelocity, Rotation2d(turningMotorEncoder.position))
        set(desiredState) {
            super.state = desiredState
            turningMotorEncoder.position = desiredState.angle.radians
            driveVelocity = desiredState.speedMetersPerSecond
        }

    override val position: SwerveModulePosition
        get() = SwerveModulePosition(drivePosition, Rotation2d(turningMotorEncoder.position))

    override fun update() {
        val currTime = Timer.getFPGATimestamp()
        drivePosition += driveVelocity * (currTime - prevTime)
        prevTime = currTime
    }
}
