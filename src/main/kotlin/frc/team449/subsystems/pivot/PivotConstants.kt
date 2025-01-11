package frc.team449.subsystems.pivot

import kotlin.math.PI

object PivotConstants {
  const val MOTOR_ID = 2 // TODO(Change motor ID.)

  const val GEARING = 1.0 / 75.0
  const val UPR = 1.0

  // Simulation Constants
  const val MOMENT_OF_INERTIA = 0.57125221 + 0.085
  const val ARM_LENGTH = 1.0
  const val MIN_ANGLE = PI / 12
  const val MAX_ANGLE = 5 * PI / 9

  const val TOLERANCE = 0.05 // TODO(Adjust tolerance.)
}
