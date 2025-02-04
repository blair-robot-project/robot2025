package frc.team449.subsystems.drive.swerve.sim

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import frc.team449.subsystems.drive.swerve.SwerveConstants
import frc.team449.subsystems.drive.swerve.SwerveModule
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig
import org.ironmaple.simulation.motorsims.SimulatedMotorController

class SwerveModuleKrakenSim(name: String, moduleIn: SwerveModuleSimulation, locationIn: Translation2d) : SwerveModule {
  override val location: Translation2d = locationIn
  public val module: SwerveModuleSimulation = moduleIn/* = SwerveModuleSimulation(
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
    )
  )*/

  val drive: SimulatedMotorController.GenericMotorController
  val turn: SimulatedMotorController.GenericMotorController

  init {
    drive = module
      .useGenericMotorControllerForDrive()
      .withCurrentLimit(SwerveConstants.DRIVE_FOC_CURRENT_LIMIT)
    turn = module
      .useGenericControllerForSteer()
      .withCurrentLimit(SwerveConstants.STEERING_CURRENT_LIM)
  }

  /*override val location: Translation2d = Translation2d(
    0.0,0.0
  );*/

  override val desiredState: SwerveModuleState = SwerveModuleState(
    0.0,
    Rotation2d()
  );

  /** The module's [SwerveModuleState], containing speed and angle. */
  override var state: SwerveModuleState
    get() = module.currentState
    set(value) {
      module.currentState.angle = value.angle;
      module.currentState.speedMetersPerSecond = value.speedMetersPerSecond;
    }


  /** The module's [SwerveModulePosition], containing distance and angle. */
  override val position: SwerveModulePosition
    get() = SwerveModulePosition(
      // Calculate distance from drive position
      Meters.of(module.driveWheelFinalPosition.`in`(Radians) * SwerveConstants.WHEEL_RADIUS),
      module.steerAbsoluteFacing
    )


  override fun setVoltage(volts: Double) {
    drive.requestVoltage(Volts.of(volts))
  }

  /** Set module speed to zero but keep module angle the same. */
  override fun stop() {
    drive.requestVoltage(Volts.of(0.0));}

  override fun update() {}
}