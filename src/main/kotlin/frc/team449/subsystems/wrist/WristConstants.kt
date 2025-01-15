package frc.team449.subsystems.wrist

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

  const val KS = 0.0
  const val KV = 0.0
  const val KA = 0.0

  const val KP = 1.0
  const val KI = 0.0
  const val KD = 0.0

  const val CRUISE_VEL = 1.0
  const val MAX_ACCEL = 1.0
}
