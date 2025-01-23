package frc.team449.subsystems.drive.swerve.sim

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import frc.team449.subsystems.drive.swerve.SwerveModule
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig

abstract class SwerveModuleKrakenSim: SwerveModule {

  val module: SwerveModuleSimulation = SwerveModuleSimulation(
    SwerveModuleSimulationConfig(
      DCMotor.getKrakenX60(1),
      DCMotor.getKrakenX60(1),
      0.0,
      0.0,
      Volts.of(12.0),
      Volts.of(12.0),
      Meters.of(1.1),
      KilogramSquareMeters.of(12.0),
      0.0
    )
  )

  val drive = module
    .useGenericMotorControllerForDrive()
    .withCurrentLimit(Amps.of(60.0));
  val turn = module
    .useGenericControllerForSteer()
    .withCurrentLimit(Amps.of(20.0));
  //val location = Translation2d();

  //val desiredState = SwerveModuleState();

  /** The module's [SwerveModuleState], containing speed and angle. */
  //var state = SwerveModuleState();

  /** The module's [SwerveModulePosition], containing distance and angle. */
  //val position = SwerveModulePosition();


  override fun setVoltage(volts: Double) {
    drive.requestVoltage(Volts.of(volts));
  //SwerveModuleSimulation.driveMotor.requestVoltage(voltage);
  }

  /** Set module speed to zero but keep module angle the same. */
  fun stop() {
    drive.requestVoltage(Volts.of(0.));}

  override fun update() {}
}