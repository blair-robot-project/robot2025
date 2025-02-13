package frc.team449.subsystems.drive.swerve.sim

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Units.*
import frc.team449.subsystems.drive.swerve.SwerveConstants
import frc.team449.subsystems.drive.swerve.SwerveModule
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation
import org.ironmaple.simulation.motorsims.SimulatedMotorController
import kotlin.math.PI
import kotlin.math.abs

// MapleSim Swerve Module Sim based on SwerveModuleKraken
class SwerveModuleSim(
  private val name: String,
  val module: SwerveModuleSimulation,
  val turnController: PIDController,
  override val location: Translation2d
): SwerveModule {
  init {
    turnController.enableContinuousInput(.0, 2 * PI)
    turnController.reset()
  }

  val drive: SimulatedMotorController.GenericMotorController = module
    .useGenericMotorControllerForDrive()
    .withCurrentLimit(SwerveConstants.DRIVE_FOC_CURRENT_LIMIT)

  val turn: SimulatedMotorController.GenericMotorController = module
    .useGenericControllerForSteer()
    .withCurrentLimit(SwerveConstants.STEERING_CURRENT_LIM)

  override val desiredState: SwerveModuleState = SwerveModuleState(
    0.0,
    Rotation2d()
  )

  /** The module's [SwerveModuleState], containing speed and angle. */
  override var state: SwerveModuleState
    get() {
      return SwerveModuleState(
        module.driveEncoderUnGearedSpeed.`in`(RadiansPerSecond),
        Rotation2d(module.steerRelativeEncoderPosition)
      )
    }
    set(desState) {
      if (abs(desState.speedMetersPerSecond) < .001) {
        stop()
        return
      }
      /** Ensure the module doesn't turn more than 90 degrees. */
      desState.optimize(Rotation2d(module.steerRelativeEncoderPosition))

      turnController.setpoint = desState.angle.radians
      desiredState.speedMetersPerSecond = desState.speedMetersPerSecond
      desiredState.angle = desState.angle
    }

  /** The module's [SwerveModulePosition], containing distance and angle. */
  override val position: SwerveModulePosition
    get() = SwerveModulePosition(
      // Calculate distance from drive position
      Meters.of(module.driveWheelFinalPosition.`in`(Radians) * SwerveConstants.WHEEL_RADIUS),
      module.steerAbsoluteFacing
    )


  override fun setVoltage(volts: Double) {
    desiredState.speedMetersPerSecond = 0.0
    turnController.setpoint = 0.0

    turn.requestVoltage(Volts.of(turnController.calculate(module.steerRelativeEncoderPosition.`in`(Radians))))
    drive.requestVoltage(Volts.of(volts))
  }

  /** Set module speed to zero but keep module angle the same. */
  override fun stop() {
    turnController.setpoint = module.steerRelativeEncoderPosition.`in`(Radians)
    desiredState.speedMetersPerSecond = 0.0
  }

  override fun update() {
    // MapleSim handles updating
  }

  companion object {
    fun createModuleSim(
      name: String,
      module: SwerveModuleSimulation,
      location: Translation2d
      ): SwerveModuleSim {
      return SwerveModuleSim(
        name,
        module,
        PIDController(
          SwerveConstants.TURN_KP,
          SwerveConstants.TURN_KI,
          SwerveConstants.TURN_KD
        ),
        location
      )
    }
  }
}