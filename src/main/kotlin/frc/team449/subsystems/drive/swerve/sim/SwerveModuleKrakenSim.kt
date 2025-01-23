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
}