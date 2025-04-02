package frc.team449.subsystems.voltage

import edu.wpi.first.units.Units.Amps
import frc.team449.subsystems.superstructure.elevator.ElevatorConstants

object VoltageDistributionConstants {
  // motor channels
  val pivotChannel = 0 // placeholder
  val pivotChannel2 = 1 // placeholder

  val elevatorChannel = 2 // placeholder
  val elevatorChannel2 = 4 // placeholder

  val wristChannel = 3 // placeholder

  val driveChannel = 10

  val DRIVE_SUPPLY_LIMIT = Amps.of(52.5)
  val DRIVE_STATOR_LIMIT = Amps.of(105.0)
  val STEERING_CURRENT_LIM = Amps.of(40.0)

  val MAX_CURRENT = 120
}