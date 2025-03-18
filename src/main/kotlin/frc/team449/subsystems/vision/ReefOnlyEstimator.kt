package frc.team449.subsystems.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.*
import edu.wpi.first.wpilibj.DriverStation
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.Optional
import kotlin.math.abs

/**
 * This class uses normal multi-tag PNP and lowest ambiguity using the gyro rotation
 *  for the internal cam-to-tag transform as a fallback strategy
 */
class ReefOnlyEstimator(
  private val tagLayout: AprilTagFieldLayout,
  val camera: PhotonCamera,
  private val robotToCam: Transform3d
) : PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam) {

  private val reportedErrors: HashSet<Int> = HashSet()
  private var driveHeading: Rotation2d? = null
  private var lastPose: Pose3d? = null

  fun updatePose(cameraResult: PhotonPipelineResult?, currPose: Pose2d): Optional<EstimatedRobotPose> {
    driveHeading = currPose.rotation
    lastPose = Pose3d(currPose.x, currPose.y, 0.0, Rotation3d(0.0, 0.0, currPose.rotation.radians))

    // Time in the past -- give up, since the following if expects times > 0
    if (cameraResult!!.timestampSeconds < 0) {
      return Optional.empty()
    }

    // If the pose cache timestamp was set, and the result is from the same timestamp, return an
    // empty result
    if (poseCacheTimestampSeconds > 0 &&
      abs(poseCacheTimestampSeconds - cameraResult.timestampSeconds) < 1e-6
    ) {
      return Optional.empty()
    }

    // Remember the timestamp of the current result used
    poseCacheTimestampSeconds = cameraResult.timestampSeconds

    // If no targets seen, trivial case -- return empty result
    return if (!cameraResult.hasTargets()) {
      Optional.empty()
    } else {
      multiTagOnCoprocStrategy(cameraResult)
    }
  }

  private fun checkBest(check: Pose3d?, opt1: Pose3d?, opt2: Pose3d?): Pose3d? {
    if (check == null || opt1 == null || opt2 == null) return null
    val dist1 = check.translation.toTranslation2d().getDistance(opt1.translation.toTranslation2d())
    val dist2 = check.translation.toTranslation2d().getDistance(opt2.translation.toTranslation2d())

    return if (dist1 < dist2) {
      opt1
    } else {
      opt2
    }
  }

  private fun multiTagOnCoprocStrategy(result: PhotonPipelineResult): Optional<EstimatedRobotPose> {
    return if (result.multiTagResult.isPresent) {
      val multitagResult = result.multiTagResult.get()

      val best_tf = multitagResult.estimatedPose.best
      val best = Pose3d()
        .plus(best_tf) // field-to-camera
        .relativeTo(tagLayout.origin)
        .plus(robotToCam.inverse()) // field-to-robot

      val alternate_tf = multitagResult.estimatedPose.alt
      val alternate = Pose3d()
        .plus(alternate_tf) // field-to-camera
        .relativeTo(tagLayout.origin)
        .plus(robotToCam.inverse()) // field-to-robot

      val visionPose = checkBest(lastPose, best, alternate) ?: best

      Optional.of(
        EstimatedRobotPose(
          visionPose,
          result.timestampSeconds,
          result.getTargets(),
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
        )
      )
    } else {
      lowestAmbiguityStrat(result)
    }
  }

  /**
   * Return the estimated position of the robot with the lowest position ambiguity from a List of
   * pipeline results.
   *
   * @param result pipeline result
   * @return the estimated position of the robot in the FCS and the estimated timestamp of this
   * estimation.
   */
  private fun lowestAmbiguityStrat(result: PhotonPipelineResult): Optional<EstimatedRobotPose> {
    var lowestAmbiguityTarget: PhotonTrackedTarget? = null
    var lowestAmbiguityScore = 10.0
    for (target: PhotonTrackedTarget in result.targets) {
      val targetPoseAmbiguity = target.poseAmbiguity
      // Make sure the target is a Fiducial target.
      if (targetPoseAmbiguity != -1.0 && targetPoseAmbiguity < lowestAmbiguityScore &&
        listOf(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22).contains(target.fiducialId)
      ) {
        lowestAmbiguityScore = targetPoseAmbiguity
        lowestAmbiguityTarget = target
      }
    }

    // Although there are confirmed to be targets, none of them may be fiducial
    // targets.
    if (lowestAmbiguityTarget == null) return Optional.empty()
    val targetFiducialId = lowestAmbiguityTarget.fiducialId
    val targetPosition = tagLayout.getTagPose(targetFiducialId)
    if (targetPosition.isEmpty) {
      reportFiducialPoseError(targetFiducialId)
      return Optional.empty()
    }

    val bestPose = targetPosition
      .get()
      .transformBy(
        lowestAmbiguityTarget.bestCameraToTarget.inverse()
      )
      .transformBy(robotToCam.inverse())

    val altPose = targetPosition
      .get()
      .transformBy(
        lowestAmbiguityTarget.altCameraToTarget.inverse()
      )
      .transformBy(robotToCam.inverse())

    val visionPose = checkBest(lastPose, bestPose, altPose) ?: bestPose

    return Optional.of(
      EstimatedRobotPose(
        visionPose,
        result.timestampSeconds,
        mutableListOf(lowestAmbiguityTarget),
        PoseStrategy.LOWEST_AMBIGUITY
      )
    )
  }

  private fun reportFiducialPoseError(fiducialId: Int) {
    if (!reportedErrors.contains(fiducialId)) {
      DriverStation.reportError(
        "[PhotonPoseEstimator] Tried to get pose of unknown AprilTag: $fiducialId",
        false
      )
      reportedErrors.add(fiducialId)
    }
  }
}
