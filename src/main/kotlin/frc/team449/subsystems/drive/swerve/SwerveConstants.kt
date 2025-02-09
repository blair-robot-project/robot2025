package frc.team449.subsystems.drive.swerve

import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*
import kotlin.math.PI

object SwerveConstants {
  const val EFFICIENCY = 0.95

  const val USE_FOC = false
  const val DUTY_CYCLE_DEADBAND = 0.001


  /** Drive motor ports */
  const val DRIVE_MOTOR_FL = 21
  const val DRIVE_MOTOR_FR = 22
  const val DRIVE_MOTOR_BL = 23
  const val DRIVE_MOTOR_BR = 24

  const val TURN_MOTOR_FL = 23
  const val TURN_MOTOR_FR = 6
  const val TURN_MOTOR_BL = 13
  const val TURN_MOTOR_BR = 5

  /** Turning encoder channels */
  const val TURN_ENC_CHAN_FL = 6
  const val TURN_ENC_CHAN_FR = 7
  const val TURN_ENC_CHAN_BL = 8
  const val TURN_ENC_CHAN_BR = 9

  /** Offsets for the absolute encoders in rotations. */
  const val TURN_ENC_OFFSET_FL = 0.1348
  const val TURN_ENC_OFFSET_FR = 0.2927 + 0.5
  const val TURN_ENC_OFFSET_BL = 0.4755
  const val TURN_ENC_OFFSET_BR = 0.391 + 0.5

  /** Inverted */
  const val DRIVE_INVERTED = false
  const val TURN_INVERTED = true
  const val TURN_ENC_INVERTED = false

  /** PID gains for turning each module */
  const val TURN_KP = 0.5
  const val TURN_KI = 0.0
  const val TURN_KD = 0.0

  /** Feed forward values for driving each module */
  const val DRIVE_KS = 0.20285 + 0.02
  const val DRIVE_KV = 2.3887 + 0.2 + 0.0935
  const val DRIVE_KA = 0.43365 + 0.035 + 0.0185

  // TODO: Figure out this value
  const val STEER_KS = 0.05 / 12.0

  /** PID gains for driving each module*/
  const val DRIVE_KP = 0.475
  const val DRIVE_KI = 0.0
  const val DRIVE_KD = 0.0

  /** Drive configuration */
  val WHEEL_RADIUS = Units.inchesToMeters(1.895)
  const val DRIVE_GEARING = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0)
  val DRIVE_UPR = 2 * PI * WHEEL_RADIUS
  const val TURN_UPR = 2 * PI
  val MAX_ATTAINABLE_MK4I_SPEED = Units.feetToMeters(15.85) // (12 - DRIVE_KS) / DRIVE_KV

  val DRIVE_SUPPLY_LIMIT = Amps.of(0.0)
  val DRIVE_FOC_CURRENT_LIMIT = Amps.of(80.0)
  val DRIVE_SUPPLY_BOOST = Amps.of(55.0)
  val DRIVE_SUPPLY_BOOST_TIME = Seconds.of(0.0)
  val DRIVE_STATOR_LIMIT = Amps.of(85.0)
  val STEERING_CURRENT_LIM = Amps.of(40.0)

  val KRAKEN_UPDATE_RATE = Hertz.of(100.0)
  val VALUE_UPDATE_RATE = Hertz.of(50.0)

  const val JOYSTICK_FILTER_ORDER = 2
  const val ROT_FILTER_ORDER = 1.25
  const val SKEW_CONSTANT = 15.5

  /** Wheelbase = wheel-to-wheel distance from front to back of the robot */
  /** Trackwidth = wheel-to-wheel distance from side to side of the robot */
  val WHEELBASE = Units.inchesToMeters(27.0 - 5.25) // ex. FL to BL, aka 5.25in less than robot length
  val TRACKWIDTH = Units.inchesToMeters(27.0 - 5.25) // ex. BL to BR, aka 5.25in less than robot width
  val X_SHIFT = 0.0 // ex. if your modules aren't centered and have a shifted wheelbase
}
