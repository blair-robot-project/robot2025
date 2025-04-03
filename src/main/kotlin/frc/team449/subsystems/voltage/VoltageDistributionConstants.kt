package frc.team449.subsystems.voltage

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Units.Amps
import frc.team449.subsystems.superstructure.elevator.ElevatorConstants

object VoltageDistributionConstants {
  // motor channels
  val pivotChannel = 0 // placeholder
  val pivotChannel2 = 1 // placeholder

  val elevatorChannel = 2 // placeholder
  val elevatorChannel2 = 4 // placeholder

  val wristChannel = 3 // placeholder

  val driveChannel1 = 10 // placeholder
  val driveChannel2 = 11 // placeholder
  val driveChannel3 = 12 // placeholder
  val driveChannel4 = 13 // placeholder

  val DRIVE_SUPPLY_LIMIT = Amps.of(52.5)
  val DRIVE_STATOR_LIMIT = Amps.of(105.0)
  val STEERING_CURRENT_LIM = Amps.of(40.0)

  val MAX_CURRENT = 120

  lateinit var pivotMotor1: TalonFX
  lateinit var pivotMotor2: TalonFX

  lateinit var eleMotor1: TalonFX
  lateinit var eleMotor2: TalonFX

  lateinit var wristMotor: TalonFX
}
