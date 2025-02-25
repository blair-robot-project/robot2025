package frc.team449.subsystems.superstructure.climb

import edu.wpi.first.units.Units.Amps

object ClimbConstants {
  const val RIGHT_MOTOR_ID = 62
  const val LEFT_MOTOR_ID = 3

  val CURRENT_LIMIT = Amps.of(30.0)
  const val BRAKE_MODE = true

  const val RIGHT_INVERTED = true
  const val LEFT_INVERTED_FROM_RIGHT = true

  const val RUN_VOLTAGE = 12.0

  const val SENSOR_DIO_PORT = 1
  const val SENSOR2_DIO_PORT = 5
}
