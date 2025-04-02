package frc.team449.subsystems.superstructure.wrist

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

  const val MOTOR_ID = 41

  val TOLERANCE = Degrees.of(5.0)
  val ELEVATOR_READY = Degrees.of(105.0)
  const val GEARING = 17.0 / 576.0
  const val UPR = 2 * PI

  val INVERTED = InvertedValue.Clockwise_Positive
  val BRAKE_MODE = NeutralModeValue.Brake

  const val STATOR_LIM = 80.0
  const val SUPPLY_LIM = 40.0

  val VALUE_UPDATE_RATE: Frequency = Hertz.of(50.0)
  val REQUEST_UPDATE_RATE: Frequency = Hertz.of(100.0)

  val CLIMB_DOWN = Degrees.of(90.0)

  const val KS = 0.020
  const val KG = 0.208
  const val KV = 0.63027

  const val KP = 5.9119
  const val KI = 0.0
  const val KD = 0.096482

  val CRUISE_VEL = RotationsPerSecond.of(2.75) // should get to 2.5, max theoretical 2.95
  val MAX_ACCEL = RotationsPerSecondPerSecond.of(7.0) // should get to 3.0, max theoretical 18.842

  val STARTUP_ANGLE = Degrees.of(129.7339) // TODO: Change to back hardstop angle
}
