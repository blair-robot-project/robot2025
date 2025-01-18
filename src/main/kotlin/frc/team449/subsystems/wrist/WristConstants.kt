package frc.team449.subsystems.wrist

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Frequency
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import kotlin.math.PI

object WristConstants {
  const val ANGLE = 0.0
  const val WIDTH = 2.0
  val COLOR = Color8Bit(Color.kWhite)

  const val MOTOR_ID = 40 // TODO(Change motor ID.)

  const val TOLERANCE = 0.05
  const val GEARING = 1.0 / 300.0
  const val UPR = 2 * PI

  val INVERTED = InvertedValue.CounterClockwise_Positive
  val BRAKE_MODE = NeutralModeValue.Brake

  const val STATOR_LIM = 80.0
  const val SUPPLY_LIM = 40.0

  val VALUE_UPDATE_RATE: Frequency = Hertz.of(50.0)

  const val KS = 0.0
  const val KV = 0.0
  const val KA = 0.0

  const val KP = 1.0
  const val KI = 0.0
  const val KD = 0.0

  val CRUISE_VEL = RadiansPerSecond.of(2 * PI)
  val MAX_ACCEL = RadiansPerSecondPerSecond.of(6 * PI)
}
