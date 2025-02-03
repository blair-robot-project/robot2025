package frc.team449.subsystems.drive.swerve

import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*
import kotlin.math.PI

object SwerveConstants {
  const val EFFICIENCY = 0.95

  const val USE_FOC = false
  const val DUTY_CYCLE_DEADBAND = 0.001

  /** Drive motor ports */
  const val DRIVE_MOTOR_FL = 11
  const val DRIVE_MOTOR_FR = 5
  const val DRIVE_MOTOR_BL = 30
  const val DRIVE_MOTOR_BR = 9
  const val TURN_MOTOR_FL = 12
  const val TURN_MOTOR_FR = 6
  const val TURN_MOTOR_BL = 8
  const val TURN_MOTOR_BR = 10

  /** Turning encoder channels */
  const val TURN_ENC_CHAN_FL = 0
  const val TURN_ENC_CHAN_FR = 3
  const val TURN_ENC_CHAN_BL = 2
  const val TURN_ENC_CHAN_BR = 1

  /** Offsets for the absolute encoders in rotations. */
  const val TURN_ENC_OFFSET_FL = -0.048002
  const val TURN_ENC_OFFSET_FR = -0.388454
  const val TURN_ENC_OFFSET_BL = -0.354592 + 0.5
  const val TURN_ENC_OFFSET_BR = -0.030209 + 0.5

  /** Inverted */
  const val DRIVE_INVERTED = true
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
  const val DRIVE_KP = 0.1
  const val DRIVE_KI = 0.0
  const val DRIVE_KD = 0.0

  /** Drive configuration */
  val WHEEL_RADIUS = Units.inchesToMeters(1.895)
  const val DRIVE_GEARING = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0)
  val DRIVE_UPR = 2 * PI * WHEEL_RADIUS
  const val TURN_UPR = 2 * PI
  val MAX_ATTAINABLE_MK4I_SPEED = Units.feetToMeters(10.85) // (12 - DRIVE_KS) / DRIVE_KV

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
