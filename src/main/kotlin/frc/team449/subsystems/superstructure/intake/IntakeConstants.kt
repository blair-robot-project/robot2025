package frc.team449.subsystems.superstructure.intake

import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.measure.Current

object IntakeConstants {
  const val MOTOR_ID = 46

  val CURRENT_LIMIT: Current = Amps.of(30.0)
  const val BRAKE_MODE = true

  const val HAS_PIVOT_SIDE_IR_SENSOR = true

  const val MOTOR_INVERTED = false

  const val CORAL_INTAKE_VOLTAGE = 11.0
  const val ALGAE_INTAKE_VOLTAGE = 9.0

  const val ALGAE_HOLD_VOLTAGE = 4.15
  const val CORAL_HOLD_VOLTAGE = 0.35

  const val CORAL_OUTTAKE_VOLTAGE = 4.0
  const val CORAL_OUTTAKE_PIVOT_SIDE_VOLTAGE = -4.0
  const val L1_OUTTAKE = -2.0
  const val ALGAE_OUTTAKE_VOLTAGE = -10.0

  const val DESCORE_ALGAE_VOLTAGE = -6.0

  const val WAIT_AFTER_ALGAE_DETECTED = 0.5

  const val CORAL_SENSOR_DIO_PORT = 0
  const val ALGAE_SENSOR_DIO_PORT = 11
  const val PIVOT_SENSOR_DIO_PORT = 12

  const val READY_PIVOT_CORAL_TIME = 0.25
}
