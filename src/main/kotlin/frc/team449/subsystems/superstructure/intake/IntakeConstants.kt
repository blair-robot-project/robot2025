package frc.team449.subsystems.superstructure.intake

import edu.wpi.first.units.Units.Amps

object IntakeConstants {
  const val MOTOR_ID = 46

  val CURRENT_LIMIT = Amps.of(30.0)
  const val BRAKE_MODE = true

  const val MOTOR_INVERTED = false

  const val CORAL_INTAKE_VOLTAGE = 6.0
  const val CORAL_HOLD_VOLTAGE = 0.55

  const val CORAL_OUTTAKE_VOLTAGE = -4.0
  const val ALGAE_INTAKE_VOLTAGE = 7.0
  const val ALGAE_OUTTAKE_VOLTAGE = -10.0

  const val SENSOR_DIO_PORT = 0
}
