package frc.team449.subsystems.superstructure.climb

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.Amps

object ClimbConstants {
  const val MOTOR_ID = 49

  val CURRENT_LIMIT = Amps.of(85.0)
  val INVERTED = InvertedValue.Clockwise_Positive
  val BRAKE_MODE = NeutralModeValue.Brake

  const val RUN_VOLTAGE = 12.0
  const val HOLD_VOLTAGE = 4.0

  const val SENSOR_DIO_PORT = 5
  const val SUPPLY_LIM = 40.0
  const val STATOR_LIM = 80.0

  const val CLIMB_DEBOUNCER_WAIT = 0.4
}
