package frc.team449.subsystems.superstructure.intake

import edu.wpi.first.units.Units.Amps

object IntakeConstants {
  const val FRONT_MOTOR_ID = 46

  val CURRENT_LIMIT = Amps.of(30.0)
  const val BRAKE_MODE = true

  const val FRONT_MOTOR_INVERTED = false
  const val BACK_MOTOR_INVERTED = true

  const val CORAL_INTAKE_VOLTAGE = 6.0
  const val CORAL_HOLD_VOLTAGE = 2.0
  const val CORAL_OUTTAKE_VOLTAGE = -4.0
  const val ALGAE_INTAKE_VOLTAGE = 7.0
  const val ALGAE_OUTTAKE_VOLTAGE = -10.0

  const val SENSOR_DIO_PORT = 0
}
