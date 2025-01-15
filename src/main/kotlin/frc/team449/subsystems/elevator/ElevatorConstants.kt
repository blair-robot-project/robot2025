package frc.team449.subsystems.elevator

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Hertz
import edu.wpi.first.units.measure.Frequency
import edu.wpi.first.wpilibj.util.Color8Bit
import kotlin.math.PI

object ElevatorConstants {
  val ANGLE = Units.radiansToDegrees(PI / 12) // DEGREES
  const val WIDTH = 7.0
  const val DESIRED_WIDTH = 4.0
  val COLOR = Color8Bit(255, 0, 255)
  val DESIRED_COLOR = Color8Bit(0, 255, 0)
  const val MIN_VIS_HEIGHT = 0.2

  const val MAX_HEIGHT = 0.75
  const val MIN_HEIGHT = 0.0

  const val LEAD_MOTOR_ID = 1
  const val FOLLOWER_MOTOR_ID = 22 // TODO(Change motor ID.)

  val INVERTED = InvertedValue.CounterClockwise_Positive
  const val FOLLOWER_INVERTED_TO_MASTER = true
  val BRAKE_MODE = NeutralModeValue.Brake

  const val STATOR_LIM = 80.0
  const val SUPPLY_LIM = 40.0

  val VALUE_UPDATE_RATE: Frequency = Hertz.of(50.0)
  val REQUEST_UPDATE_RATE: Frequency = Hertz.of(100.0)

  // Simulation Constants TODO(Adjust all of these.)
  const val GEARING = 2 / 6.4
  const val PULLEY_RADIUS = 0.018415
  const val UPR = 2 * PI * PULLEY_RADIUS
  const val CARRIAGE_MASS = 9.45

  const val TOLERANCE = 0.025 // TODO(Adjust tolerance.)

  // TODO(Adjust gains.)
  const val KS = 0.0
  const val KV = 0.0
  const val KA = 0.0
  const val KG = 0.0

  const val KP = 10.0
  const val KI = 0.0
  const val KD = 0.0

  // Motion Magic
  const val CRUISE_VEL = 1.0
  const val MAX_ACCEL = 1.0
}
