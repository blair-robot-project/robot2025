package frc.team449.subsystems.superstructure.elevator

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Hertz
import edu.wpi.first.units.measure.Frequency
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.util.Color8Bit
import kotlin.math.PI

object ElevatorConstants {
  val ANGLE = Units.radiansToDegrees(PI / 12) // DEGREES
  const val WIDTH = 7.0
  const val DESIRED_WIDTH = 4.0
  val COLOR = Color8Bit(255, 0, 255)
  val DESIRED_COLOR = Color8Bit(0, 255, 0)
  const val MIN_VIS_HEIGHT = 0.2

  val SIM_MAX_HEIGHT = Units.inchesToMeters(52.5)
  val SIM_MIN_HEIGHT = 0.0

  const val LEAD_MOTOR_ID = 30
  const val FOLLOWER_MOTOR_ID = 31

  val INVERTED = InvertedValue.CounterClockwise_Positive
  const val FOLLOWER_INVERTED_TO_MASTER = true
  val BRAKE_MODE = NeutralModeValue.Brake

  const val STATOR_LIM = 80.0
  const val SUPPLY_LIM = 40.0

  val VALUE_UPDATE_RATE: Frequency = Hertz.of(50.0)
  val REQUEST_UPDATE_RATE: Frequency = Hertz.of(100.0)

  // Physical Constants TODO(Adjust all of these.)
  const val GEARING_MOTOR_TO_PULLEY = 1.0 / 4.5
  val PULLEY_RADIUS = Units.inchesToMeters(1.5 / 2)
  val UPR = 2 * PI * PULLEY_RADIUS
  val GEARING_MOTOR_TO_ELEVATOR = 1 / (GEARING_MOTOR_TO_PULLEY * UPR)
  const val CARRIAGE_MASS = 4.0

  const val TOLERANCE = 0.05 // TODO(Adjust tolerance.)

  // TODO(Adjust gains.)
  val KS = if (RobotBase.isReal()) 0.0175 else 0.0
  const val KV = 4.5112 // theoretical 4.5112
  val KG = if (RobotBase.isReal()) 0.26851 else 0.0 // was 0.26233

  const val KP = 9.8532
  const val KI = 0.0
  const val KD = 0.0015

  // Motion Magic
  const val CRUISE_VEL = 2.5 // max theoretical 2.66 m/s, max practical ? m/s from feedforward
  const val MAX_ACCEL = 6.0 // should get to 15.0, max theoretical ? m/s/s at 4kg and no gravity
}
