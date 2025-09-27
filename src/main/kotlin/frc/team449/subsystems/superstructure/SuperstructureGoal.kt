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

  private val MIN_ELEVATOR_HEIGHT = Inches.of(-0.5)

  val STOW = SuperstructureState(
    Degrees.of(40.0),
    MIN_ELEVATOR_HEIGHT,
    Radians.of(2.047607421875),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "Stow"
  )

  val L1 = SuperstructureState(
    Radians.of(0.866943359375),
    MIN_ELEVATOR_HEIGHT,
    Radians.of(-1.106455078125),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "L1"
  )

  val L2 = SuperstructureState(
    Radians.of(0.315185546875),
    MIN_ELEVATOR_HEIGHT,
    Radians.of(2.313232421875),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "L2"
  )

  val L3 = SuperstructureState(
    Radians.of(0.90576171875),
    Meters.of(0.2109375),
    Radians.of(1.28125),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "L3"
  )

  val L4 = SuperstructureState(
    Radians.of(1.2470703125) + Degrees.of(1.0),
    Meters.of(0.82666015625 + 0.02),
    Radians.of(0.25634765625),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "L4"
  )

  val L4_PREMOVE = SuperstructureState(
    L4.pivot,
    STOW.elevator,
    STOW.wrist,
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "L4 Premove"
  )

  val L2_PIVOT = SuperstructureState(
    Radians.of(1.6826171875),
    MIN_ELEVATOR_HEIGHT,
    Radians.of(2.0126953125),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "L2 Pivot"
  )

  val L3_PIVOT = SuperstructureState(
    Radians.of(1.328125),
    Meters.of(0.2255859375),
    Radians.of(2.11376953125),
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "L3 Pivot"
  )

  val L4_PIVOT = SuperstructureState(
    Radians.of(1.4248046875), // 83 deg
    Meters.of(0.90478515625),
    Radians.of(2.3525390625), // 133 deg
    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "L4 Pivot"
  )

  val GROUND_INTAKE_CORAL = SuperstructureState(
    Radian.of(-0.02239),
    MIN_ELEVATOR_HEIGHT,
    Radians.of(-0.2900390625 - 0.01-0.02),
    DriveDynamics(GROUND_INTAKE_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "Ground Intake Coral"
  )

  val CLIMB_BEFORE = SuperstructureState(
    Degrees.of(90.0),
    Meters.of(0.178891),
    Degrees.of(-70.0),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "Climb Before"
  )

  val L4_PREMOVE_PIVOT = SuperstructureState(
    Degrees.of(67.5),
    MIN_ELEVATOR_HEIGHT,
    L4_PIVOT.wrist,
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "L4 Premove Pivot"
  )

  val NET = SuperstructureState(
    Radians.of(1.365722),
    Meters.of(1.18603515625),
    Radians.of(-0.76806640625),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "Net"
  )

  val NET_PREMOVE = SuperstructureState(
    NET.pivot,
    STOW.elevator,
    STOW.wrist,
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "Net Premove"
  )

  // TODO: FIND NET PIVOT POSE
  val NET_PIVOT = SuperstructureState(
    Radians.of(1.4404296875),
    Meters.of(1.0732421875),
    Radians.of(0.69091796875),

    DriveDynamics(SCORING_SPEED, SCORING_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "Net Pivot"
  )

  // TODO: FIND PIVOT FOR PREMOVE STATE
  val NET_PREMOVE_PIVOT = SuperstructureState(
    Degrees.of(67.5),
    MIN_ELEVATOR_HEIGHT,
    NET_PIVOT.wrist,
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "Net Pivot Premove"
  )

  val ALGAE_GROUND = SuperstructureState(
    Radian.of(0.306),
    Inches.of(0.0),
    Radians.of(-1.499),
    DriveDynamics(GROUND_INTAKE_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "Algae Ground"
  )

  val L2_ALGAE_INTAKE = SuperstructureState(
    Radians.of(0.905),
    MIN_ELEVATOR_HEIGHT,
    Radians.of(-0.958),
    DriveDynamics(GROUND_INTAKE_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "L2 Algae Intake"
  )

  val STATION_INTAKE = SuperstructureState(
    Radians.of(1.11767578125),
    Meters.of(0.12548828125),
    Radians.of(-0.640869140625),
    DriveDynamics(GROUND_INTAKE_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "Station Intake"
  )

  val L3_ALGAE_INTAKE = SuperstructureState(
    Radians.of(1.155),
    Meters.of(0.402),
    Radians.of(-1.420),
    DriveDynamics(GROUND_INTAKE_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "L3 Algae Intake"
  )

  val PROC = SuperstructureState(
    Radians.of(0.20458984375),
    MIN_ELEVATOR_HEIGHT,
    Radians.of(-0.281005859375),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "Processor"
  )

  val CLIMB_SAFE = SuperstructureState(
    Radians.of(1.0928),
    Meters.of(0.1504),
    Radians.of(1.5708),
    DriveDynamics(RobotConstants.MAX_LINEAR_SPEED, RobotConstants.MAX_ACCEL, RobotConstants.MAX_ROT_SPEED),
    "Elevator Climb Safe"
  )

  data class SuperstructureState(
    var pivot: Angle,
    var elevator: Distance,
    var wrist: Angle,
    val driveDynamics: DriveDynamics,
    val name: String
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
