
package frc.team449.control.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import frc.team449.subsystems.vision.VisionConstants
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim
import java.util.Optional
import kotlin.math.abs
import kotlin.math.pow

class ApriltagCamera(
  name: String,
  tagLayout: AprilTagFieldLayout,
  robotToCam: Transform3d,
  private val visionSystemSim: VisionSystemSim?
) {

  val cam = PhotonCamera(name)

  val estimator = PhotonPoseEstimator(
    tagLayout,
    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    robotToCam
  )

  private var lastEstTimestamp = 0.0

  private var cameraSim: PhotonCameraSim? = null

  init {
    if (RobotBase.isSimulation()) {
      visionSystemSim!!.addAprilTags(tagLayout)

      val cameraProp = SimCameraProperties()
      cameraProp.setCalibration(VisionConstants.SIM_CAMERA_WIDTH_PX, VisionConstants.SIM_CAMERA_HEIGHT_PX, Rotation2d.fromDegrees(VisionConstants.SIM_FOV_DEG))
      cameraProp.setCalibError(VisionConstants.SIM_CALIB_AVG_ERR_PX, VisionConstants.SIM_CALIB_ERR_STDDEV_PX)
      cameraProp.fps = VisionConstants.SIM_FPS
      cameraProp.avgLatencyMs = VisionConstants.SIM_AVG_LATENCY
      cameraProp.latencyStdDevMs = VisionConstants.SIM_STDDEV_LATENCY

      cameraSim = PhotonCameraSim(cam, cameraProp)

      cameraSim!!.enableDrawWireframe(VisionConstants.ENABLE_WIREFRAME)

      visionSystemSim.addCamera(cameraSim, robotToCam)
    }
  }

  private fun getSimDebugField(): Field2d? {
    return if (!RobotBase.isSimulation()) null else visionSystemSim!!.debugField
  }

  fun estimatedPose(): List<Optional<EstimatedRobotPose>> {
    val results = cam.allUnreadResults

    val poses = ArrayList<Optional<EstimatedRobotPose>>()

    for (result in results) {
      val visionEst = estimator.update(result)
      val latestTimestamp = result.timestampSeconds
      val newResult = abs(latestTimestamp - lastEstTimestamp) > 1e-6
      if (RobotBase.isSimulation()) {
        visionEst.ifPresentOrElse(
          { est ->
            getSimDebugField()!!
              .getObject("VisionEstimation").pose = est.estimatedPose.toPose2d()
          }
        ) { if (newResult) getSimDebugField()!!.getObject("VisionEstimation").setPoses() }
      }

      if (newResult) {
        poses.add(visionEst)
        lastEstTimestamp = latestTimestamp
      }
    }

    return poses
  }

  fun getEstimationStdDevs(numTags: Int, avgDist: Double): Matrix<N3, N1> {
    var estStdDevs = VisionConstants.SINGLE_TAG_TRUST.copy()

    if (numTags == 0) return estStdDevs

    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = VisionConstants.MULTI_TAG_TRUST.copy()

//    println(estStdDevs.times(
//      1 + avgDist.pow(VisionConstants.ORDER) * VisionConstants.PROPORTION
//    )[0, 0])

    // Increase std devs based on (average) distance
    return estStdDevs.times(
      1 + avgDist.pow(VisionConstants.ORDER) * VisionConstants.PROPORTION
    )
  }

  private val reefAprilTagIDs = setOf(6, 7, 8, 9, 10, 11, 17)

  fun tagAim() {
    val results = cam.allUnreadResults
    if (!results.isEmpty()) {
      var result = results.get(results.size - 1);
      if (result.hasTargets()) {

      }
    }

    fun simulationPeriodic(robotSimPose: Pose2d?) {
      visionSystemSim!!.update(robotSimPose)
    }
  }
}
