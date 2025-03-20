package frc.team449.commands.driveAlign

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.subsystems.FieldConstants
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.vision.PoseSubsystem
import java.util.Optional
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.min

/**
 * @param drive The holonomic drive you want to align with
 */
class SimpleReefAlign(
  private val drive: SwerveDrive,
  private val poseSubsystem: PoseSubsystem,
  translationSpeedLim: Double = 0.65 * RobotConstants.MAX_LINEAR_SPEED,
  translationAccelLim: Double = 2.5,
  headingSpeedLim: Double = PI,
  headingAccelLim: Double = 4 * PI,
  translationPID: Triple<Double, Double, Double> = Triple(6.25, 0.0, 0.0),
  headingPID: Triple<Double, Double, Double> = Triple(6.5, 0.0, 0.0),
  private val translationTolerance: Double = Units.inchesToMeters(0.675),
  private val headingTolerance: Double = Units.degreesToRadians(0.75),
  private val speedTol: Double = 0.075,
  private val speedTolRot: Double = PI / 16,
  private val ffMinRadius: Double = 0.2,
  private val ffMaxRadius: Double = 0.6,
  private val leftOrRight: Optional<FieldConstants.ReefSide> = Optional.empty()
) : Command() {
  init {
    addRequirements(drive)
  }

  private val translationController = ProfiledPIDController(
    translationPID.first,
    translationPID.second,
    translationPID.third,
    TrapezoidProfile.Constraints(translationSpeedLim, translationAccelLim)
  )

  private val headingController = ProfiledPIDController(
    headingPID.first,
    headingPID.second,
    headingPID.third,
    TrapezoidProfile.Constraints(headingSpeedLim, headingAccelLim)
  )

  var targetPose: Pose2d = Pose2d()

  override fun initialize() {
    headingController.enableContinuousInput(-PI, PI)

    // Set tolerances from the given pose tolerance
    translationController.setTolerance(translationTolerance, speedTol)
    headingController.setTolerance(headingTolerance, speedTolRot)

    val currentPose = poseSubsystem.pose

    if (leftOrRight.isEmpty) {
      targetPose = if (FieldConstants.REEF_LOCATIONS.isNotEmpty()) currentPose.nearest(FieldConstants.REEF_LOCATIONS) else Pose2d()
    } else {
      val closestReef = if (FieldConstants.REEF_LOCATIONS.isNotEmpty()) currentPose.nearest(FieldConstants.REEF_CENTER_LOCATIONS) else Pose2d()
      val index = FieldConstants.REEF_CENTER_LOCATIONS.indexOf(closestReef)

      targetPose = FieldConstants.REEF_LOCATIONS[2 * index + if (leftOrRight.get() == FieldConstants.ReefSide.LEFT) 0 else 1]
    }

    val fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(
      drive.currentSpeeds.vxMetersPerSecond,
      drive.currentSpeeds.vyMetersPerSecond,
      drive.currentSpeeds.omegaRadiansPerSecond,
      currentPose.rotation
    )

    translationController.reset(
      currentPose.translation.getDistance(targetPose.translation),
      min(
        0.0,
        -Translation2d(fieldRelative.vxMetersPerSecond, fieldRelative.vyMetersPerSecond)
          .rotateBy(
            targetPose
              .translation
              .minus(currentPose.translation)
              .angle
              .unaryMinus()
          )
          .x
      )
    )

    headingController.reset(
      currentPose.rotation.radians,
      fieldRelative.omegaRadiansPerSecond
    )
  }

  override fun execute() {
    val currentPose: Pose2d = poseSubsystem.pose

    val currentDistance = currentPose.translation.getDistance(targetPose.translation)
    val ffScaler = MathUtil.clamp(
      (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),
      0.0,
      1.0
    )

    var driveVelocityScalar: Double = (translationController.setpoint.velocity * ffScaler + translationController.calculate(currentDistance, 0.0))
    if (currentDistance < translationController.positionTolerance) {
      driveVelocityScalar = 0.0
    }

    val headingError = currentPose.rotation.minus(targetPose.rotation).radians
    var headingVelocity: Double = (
      headingController.setpoint.velocity * ffScaler + headingController.calculate(
        currentPose.rotation.radians,
        targetPose.rotation.radians
      )
      )

    if (abs(headingError) < headingController.positionTolerance) {
      headingVelocity = 0.0
    }

    // 254 math
    val driveVelocity = Pose2d(
      0.0,
      0.0,
      currentPose.translation.minus(targetPose.translation).angle
    ).transformBy(Transform2d(driveVelocityScalar, 0.0, Rotation2d())).translation
    val speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      driveVelocity.x,
      driveVelocity.y,
      headingVelocity,
      currentPose.rotation
    )
    drive.set(speeds)
  }

  override fun isFinished(): Boolean {
    return translationController.atGoal() && headingController.atGoal()
  }

  override fun end(interrupted: Boolean) {
    drive.stop()
  }
}
