package frc.team449.subsystems.elevator

import edu.wpi.first.math.util.Units
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


  const val MOTOR_ID = 1 // TODO(Change motor ID.)

  // Simulation Constants TODO(Adjust all of these.)
  const val GEARING = 2 / 6.4
  const val PULLEY_RADIUS = 0.018415
  const val UPR = 0.11
  const val CARRIAGE_MASS = 9.45

  const val TOLERANCE = 0.05 // TODO(Adjust tolerance.)

  // TODO(Adjust gains.)
  const val KS = 0.0
  const val KV = 0.0
  const val KA = 0.0
  const val KG = 0.0

  const val KP = 1.0
  const val KI = 0.0
  const val KD = 0.0
}
