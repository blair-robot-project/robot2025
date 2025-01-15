package frc.team449.subsystems.pivot

import kotlin.math.PI

object PivotConstants {
  const val MOTOR_ID = 2 // TODO(Change motor ID.)

  // TODO(Adjust gearing and UPR.)
  const val GEARING = 1.0 / 75.0
  const val UPR = 2.0 * PI * (26.0 / 36.0)

  // Simulation Constants
  const val MOMENT_OF_INERTIA = 0.57125221 + 0.085
  const val ARM_LENGTH = 1.0
  const val MIN_ANGLE = PI / 12
  const val MAX_ANGLE = 5 * PI / 9

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

  const val CRUISE_VEL = 1.0
  const val MAX_ACCEL = 1.0
}
