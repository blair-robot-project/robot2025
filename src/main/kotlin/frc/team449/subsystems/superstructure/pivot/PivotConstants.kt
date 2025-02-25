package frc.team449.subsystems.superstructure.pivot

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Frequency
import edu.wpi.first.wpilibj.Encoder
import kotlin.math.PI

object PivotConstants {
  const val LEAD_MOTOR_ID = 10 // left side
  const val FOLLOWER_MOTOR_ID = 11

  // TODO(Adjust gearing and UPR.)
  const val GEARING = 1.0 / 252.0
  const val UPR = 2.0 * PI

  // Simulation Constants
  const val MOMENT_OF_INERTIA = 0.57125221 + 0.085
  const val ARM_LENGTH = 1.0
  const val MIN_ANGLE = PI / 12
  const val MAX_ANGLE = 5 * PI / 9

  val STARTING_ANGLE = Degrees.of(0.0)

  val INVERTED = InvertedValue.Clockwise_Positive
  const val FOLLOWER_INVERTED_TO_MASTER = true
  val BRAKE_MODE = NeutralModeValue.Brake

  val CLIMB_MIN_ANGLE = Degrees.of(10.0)
  val CLIMB_VOLTAGE = Volts.of(-3.0)

  const val STATOR_LIM = 80.0
  const val SUPPLY_LIM = 40.0

  val VALUE_UPDATE_RATE: Frequency = Hertz.of(50.0)
  val REQUEST_UPDATE_RATE: Frequency = Hertz.of(100.0)

  val TOLERANCE = Degrees.of(5.0) // TODO(Adjust tolerance.)

  const val KP = 10.4048
  const val KI = 0.0
  const val KD = 0.048099

  // TODO(Adjust gains.)
  const val KS = 0.085813
  const val KV = 4.4941
  const val KG_MIN = 0.13706 // (KG when elevator is fully retracted.)
  const val KG_MAX = 0.52576 // (KG when elevator is fully extended.)
  const val KG_MAX_EXTENSION = 1.36934

  val CRUISE_VEL = RotationsPerSecond.of(0.325) // max theoretical 0.392
  val MAX_ACCEL = RotationsPerSecondPerSecond.of(0.75) // 5.0, heavily limited by robot tipping

  val RESET_ENC_LIMIT = Degrees.of(0.05)

  /** Encoder Values */
  const val ABS_ENC_DIO_PORT = 4
  const val ENC_INVERTED = true
  val ABS_RANGE = Pair(-0.25, 0.75)
  const val ENC_RATIO = 2 * PI * (1.0 / 2.0)
  const val ABS_OFFSET = -2.310589 / ENC_RATIO
  val QUAD_ENCODER = Encoder(2, 3)
  const val ENC_CPR = 2048
  const val SAMPLES_TO_AVERAGE = 127
}
