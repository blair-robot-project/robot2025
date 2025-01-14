package frc.team449.commands.driveAlign

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.vision.PoseSubsystem
import kotlin.math.PI
import kotlin.math.hypot

/**
 * @param drive The holonomic drive you want to align with
 * @param targetPose The pose you want to drive up to
 * @param tolerance The allowed tolerance from the targetPose
 */
class PIDPoseAlign(
  private val drive: SwerveDrive,
  private val poseSubsystem: PoseSubsystem,
  private val targetPose: Pose2d,
  private val translationSpeedLim: Double,
  private val headingSpeedLim: Double,
  private val xPID: PIDController = PIDController(7.5, 0.0, 0.0),
  private val yPID: PIDController = PIDController(7.5, 0.0, 0.0),
  private val headingPID: PIDController = PIDController(5.0, 0.0, 0.0),
  private val tolerance: Pose2d = Pose2d(0.05, 0.05, Rotation2d.fromDegrees(3.0)),
  private val speedTol: Double = 0.10,
  private val speedTolRot: Double = PI / 16
) : Command() {
  init {
    addRequirements(drive)
  }

  override fun initialize() {
    headingPID.enableContinuousInput(-PI, PI)

    // Set tolerances from the given pose tolerance
    xPID.setTolerance(tolerance.x)
    yPID.setTolerance(tolerance.y)
    headingPID.setTolerance(tolerance.rotation.radians)

    xPID.setpoint = targetPose.x
    yPID.setpoint = targetPose.y
    headingPID.setpoint = targetPose.rotation.radians

    xPID.reset()
    yPID.reset()
    headingPID.reset()
  }

  override fun execute() {
    val currPose = poseSubsystem.pose

    // Calculate the feedback for X, Y, and theta using their respective controllers
    val xFeedback = MathUtil.clamp(xPID.calculate(currPose.x), -translationSpeedLim, translationSpeedLim)
    val yFeedback = MathUtil.clamp(yPID.calculate(currPose.y), -translationSpeedLim, translationSpeedLim)
    val headingFeedback = MathUtil.clamp(headingPID.calculate(poseSubsystem.heading.radians), -headingSpeedLim, headingSpeedLim)

    drive.set(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        xFeedback,
        yFeedback,
        headingFeedback,
        currPose.rotation
      )
    )
  }

  override fun isFinished(): Boolean {
    return xPID.atSetpoint() && yPID.atSetpoint() && headingPID.atSetpoint() &&
      hypot(
        drive.currentSpeeds.vxMetersPerSecond,
        drive.currentSpeeds.vyMetersPerSecond
      ) < speedTol &&
      drive.currentSpeeds.omegaRadiansPerSecond < speedTolRot
  }

  override fun end(interrupted: Boolean) {
    drive.stop()
  }
}
