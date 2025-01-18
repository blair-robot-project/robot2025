package frc.team449.subsystems.pivot

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Frequency
import kotlin.math.PI

object PivotConstants {
  const val LEAD_MOTOR_ID = 2 // TODO(Change motor ID.)
  const val FOLLOWER_MOTOR_ID = 33

  // TODO(Adjust gearing and UPR.)
  const val GEARING = 1.0 / 75.0
  const val UPR = 2.0 * PI * (26.0 / 36.0)

  // Simulation Constants
  const val MOMENT_OF_INERTIA = 0.57125221 + 0.085
  const val ARM_LENGTH = 1.0
  const val MIN_ANGLE = PI / 12
  const val MAX_ANGLE = 5 * PI / 9

  val INVERTED = InvertedValue.CounterClockwise_Positive
  const val FOLLOWER_INVERTED_TO_MASTER = true
  val BRAKE_MODE = NeutralModeValue.Brake

  const val STATOR_LIM = 80.0
  const val SUPPLY_LIM = 40.0

  val VALUE_UPDATE_RATE: Frequency = Hertz.of(50.0)
  val REQUEST_UPDATE_RATE: Frequency = Hertz.of(100.0)

  const val TOLERANCE = 0.05 // TODO(Adjust tolerance.)

  const val KP = 1.0
  const val KI = 0.0
  const val KD = 0.0

  // TODO(Adjust gains.)
  const val KS = 0.0
  const val KV = 0.0
  const val KA = 0.0
  const val KG_MIN = 0.0 // (KG when elevator is fully retracted.)
  const val KG_MAX = 0.0 // TODO(KG when elevator is fully extended.)

  val CRUISE_VEL = RadiansPerSecond.of(PI)
  val MAX_ACCEL = RadiansPerSecondPerSecond.of(6 * PI)
}
