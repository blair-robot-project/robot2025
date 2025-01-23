package frc.team449.subsystems.drive.swerve.sim

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import frc.team449.subsystems.drive.swerve.SwerveConstants
import frc.team449.subsystems.drive.swerve.SwerveModule
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig
import org.ironmaple.simulation.motorsims.SimulatedMotorController

abstract class SwerveModuleKrakenSim: SwerveModule {
  val module: SwerveModuleSimulation = SwerveModuleSimulation(
    SwerveModuleSimulationConfig(
      DCMotor.getKrakenX60(1),
      DCMotor.getKrakenX60(1),
      SwerveConstants.DRIVE_GEARING,
      0.0,
      SwerveConstants.DRIVE_FRICTION_VOLTAGE,
      SwerveConstants.TURN_FRICTION_VOLTAGE,
      Meters.of(SwerveConstants.WHEEL_RADIUS),
      KilogramSquareMeters.of(12.0),
      SwerveConstants.WHEEL_COEFFICIENT_OF_FRICTION
    )
  )

  val drive: SimulatedMotorController = module.useGenericMotorControllerForDrive()

  val steer: SimulatedMotorController = module.useGenericControllerForSteer()


}