package frc.team449.subsystems.superstructure.intake

import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Hertz

object IntakeConstants {
  const val FRONT_MOTOR_ID = 10 // TODO(Change motor ID.)
  const val BACK_MOTOR_ID = 11 // TODO(Change motor ID.)

  val STATOR_LIMIT = Amps.of(60.0)
  val SUPPLY_LIMIT = Amps.of(30.0)

  val VALUE_UPDATE_FREQ = Hertz.of(10.0)

  const val FRONT_MOTOR_INVERTED = false
  const val BACK_MOTOR_INVERTED = true

  const val CORAL_INTAKE_VOLTAGE = 3.0
  const val CORAL_OUTTAKE_VOLTAGE = -2.0
  const val ALGAE_INTAKE_VOLTAGE = 7.0
  const val ALGAE_OUTTAKE_VOLTAGE = -10.0
}
