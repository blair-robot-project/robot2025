package frc.team449.subsystems.superstructure

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveDrive

object SuperstructureGoal {
  /** TODO: All placeholder guesses, need actual values */
  private const val SCORING_SPEED = 2.6329
  private const val SCORING_ACCEL = 12.5

  private const val GROUND_INTAKE_SPEED = 3.5804

  private val MIN_ELEVATOR_HEIGHT = Inches.of(-0.35)

  val STOW = SuperstructureState(
    Degrees.of(40.0),
    MIN_ELEVATOR_HEIGHT,
    Radians.of(2.047607421875),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L1 = SuperstructureState(
    Radians.of(0.644),
    MIN_ELEVATOR_HEIGHT,
    Radians.of(-0.902),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L2 = SuperstructureState(
    Radians.of(-0.0168),
    MIN_ELEVATOR_HEIGHT,
    Radians.of(2.1213),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L3 = SuperstructureState(
    Radians.of(0.912),
    Meters.of(0.170),
    Radians.of(1.269),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L4 = SuperstructureState(
    Radians.of(1.208008),
    Meters.of(0.8537597),
    Radians.of(0.256591),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L4_PREMOVE = SuperstructureState(
    L4.pivot,
    STOW.elevator,
    STOW.wrist,
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L2_PIVOT = SuperstructureState(
    Radians.of(1.211914),
    MIN_ELEVATOR_HEIGHT,
    Radians.of(1.503662),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L3_PIVOT = SuperstructureState(
    Radians.of(1.326904296875),
    Meters.of(0.252197265625),
    Radians.of(2.11376953125), // 112 deg
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L4_PIVOT = SuperstructureState(
    Radians.of(1.467041015625), // 83 deg
    Meters.of(0.929443359375),
    Radians.of(2.28857421875), // 133 deg
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val GROUND_INTAKE_CORAL = SuperstructureState(
    Radian.of(-0.02239 - 0.01),
    Inches.of(0.0),
    Radians.of(-0.372558 + 0.0249),
    DriveDynamics(GROUND_INTAKE_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val CLIMB_BEFORE = SuperstructureState(
    Degrees.of(90.0),
    Meters.of(0.178891),
    Degrees.of(-70.0),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

//  // OLD L2 ALGAE DESCORE
//  val L2_ALGAE_DESCORE = SuperstructureState(
//    Degrees.of(53.449538),
//    Inches.of(7.694820-2)- MIN_ELEVATOR_HEIGHT,
//    Degrees.of(-90.0), // true angle is -84.87
//    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
//  )
//
//  // OLD L3 ALGAE DESCORE
//  val L3_ALGAE_DESCORE = SuperstructureState(
//    Radians.of(0.958984),
//    Meters.of(0.291016),
//    Radians.of(-1.064941),
//    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
//  )

  val L4_PREMOVE_PIVOT = SuperstructureState(
    Degrees.of(67.5),
    MIN_ELEVATOR_HEIGHT,
    L4_PIVOT.wrist,
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val NET = SuperstructureState(
    Radians.of(1.289),
    Meters.of(1.048), // 1.048
    Radians.of(-0.072),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val NET_PREMOVE = SuperstructureState(
    NET.pivot,
    STOW.elevator,
    STOW.wrist,
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  // TODO: FIND NET PIVOT POSE
  val NET_PIVOT = SuperstructureState(
    Radians.of(1.360),
    Meters.of(1.055),
    Radians.of(0.658), // -138.5
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  // TODO: FIND PIVOT FOR PREMOVE STATE
  val NET_PREMOVE_PIVOT = SuperstructureState(
    Degrees.of(67.5),
    MIN_ELEVATOR_HEIGHT,
    NET_PIVOT.wrist,
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val ALGAE_GROUND = SuperstructureState(
    Radian.of(0.306),
    Inches.of(0.0),
    Radians.of(-1.499),
    DriveDynamics(GROUND_INTAKE_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L2_ALGAE_INTAKE = SuperstructureState(
    Radians.of(0.905),
    MIN_ELEVATOR_HEIGHT,
    Radians.of(-0.958),
    DriveDynamics(GROUND_INTAKE_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val STATION_INTAKE = SuperstructureState(
    Radians.of(1.11767578125),
    Meters.of(0.12548828125),
    Radians.of(-0.640869140625),
    DriveDynamics(GROUND_INTAKE_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val L3_ALGAE_INTAKE = SuperstructureState(
    Radians.of(1.155),
    Meters.of(0.402),
    Radians.of(-1.420),
    DriveDynamics(GROUND_INTAKE_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val PROC = SuperstructureState(
    Radians.of(0.20458984375),
    MIN_ELEVATOR_HEIGHT,
    Radians.of(-0.281005859375),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  val CLIMB_SAFE = SuperstructureState(
    Radians.of(1.0928),
    Meters.of(0.1504),
    Radians.of(1.5708),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED)
  )

  data class SuperstructureState(
    var pivot: Angle,
    var elevator: Distance,
    var wrist: Angle,
    val driveDynamics: DriveDynamics
  )

  data class DriveDynamics(
    val maxSpeed: Double,
    val maxAccel: Double,
    val maxRotSpeed: Double
  )

  fun applyDriveDynamics(drive: SwerveDrive, dynamics: DriveDynamics) {
    drive.maxLinearSpeed = dynamics.maxSpeed
    drive.accel = dynamics.maxAccel
    drive.maxRotSpeed = dynamics.maxRotSpeed
  }
}
