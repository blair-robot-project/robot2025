package frc.team449.subsystems.superstructure.pivot

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Frequency
import edu.wpi.first.wpilibj.Encoder
import kotlin.math.PI

object PivotConstants {
  const val LEAD_MOTOR_ID = 10 // TODO(Change motor ID.)
  const val FOLLOWER_MOTOR_ID = 11


  // TODO(Adjust gearing and UPR.)
  const val GEARING = 1.0 / 75.0
  const val UPR = 2.0 * PI

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
  const val KG_MIN = 0.0 // (KG when elevator is fully retracted.)
  const val KG_MAX = 0.0 // TODO(KG when elevator is fully extended.)

  val CRUISE_VEL = RotationsPerSecond.of(0.3)
  val MAX_ACCEL = RotationsPerSecondPerSecond.of(5.0)

  val RESET_ENC_LIMIT = Degrees.of(0.5)

  /** Encoder Values */
  const val ABS_ENC_DIO_PORT = 4
  const val ABS_OFFSET = 0.0
  const val ENC_INVERTED = false
  val ABS_RANGE = Pair(-0.25, 0.75)
  const val ENC_RATIO = 2 * PI / (30.0 / 30.0)
  val QUAD_ENCODER = Encoder(5, 10)
  const val ENC_CPR = 2048
  const val SAMPLES_TO_AVERAGE = 127
}
