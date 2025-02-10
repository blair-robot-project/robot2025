package frc.team449.subsystems.superstructure.climb

import edu.wpi.first.units.Units.Amps

object ClimbConstants {
  const val RIGHT_MOTOR_ID = 12 // TODO(Change motor ID.)
  const val LEFT_MOTOR_ID = 13 // TODO(Change motor ID.)

  val CURRENT_LIMIT = Amps.of(30.0)
  const val BRAKE_MODE = true

  const val RIGHT_INVERTED = false
  const val LEFT_INVERTED_FROM_RIGHT = true

  const val RUN_VOLTAGE = 6.0

  const val SENSOR_DIO_PORT = 12
}
