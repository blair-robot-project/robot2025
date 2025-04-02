package frc.team449.subsystems.vision

import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Filesystem
import frc.team449.control.vision.ApriltagCamera
import frc.team449.subsystems.vision.interpolation.InterpolatedVisionDataset
import org.photonvision.estimation.TargetModel
import org.photonvision.simulation.VisionSystemSim

/** Constants that have anything to do with vision */
object VisionConstants {
  /** How the tags are laid out on the field (their locations and ids) */
  private val TEST_TAG_LAYOUT = AprilTagFieldLayout(
    listOf(
      AprilTag(3, Pose3d())
    ),
    16.4846,
    8.1026
  )

//  val TAG_LAYOUT: AprilTagFieldLayout = TEST_TAG_LAYOUT

  /** WPILib's AprilTagFieldLayout for the 2025 Reefscape Game */
  val TAG_LAYOUT: AprilTagFieldLayout = AprilTagFieldLayout(Filesystem.getDeployDirectory().absolutePath + "/reef_only.json")

  /** Robot to Camera distance */
  val front = Transform3d(
    Translation3d(Units.inchesToMeters(-4.0), Units.inchesToMeters(0.0), Units.inchesToMeters(8.5)),
    Rotation3d(0.0, Units.degreesToRadians(-17.5), Units.degreesToRadians(0.0))
  )

  val back_left = Transform3d(
    Translation3d(Units.inchesToMeters(-8.0), Units.inchesToMeters(11.0), Units.inchesToMeters(9.0)),
    Rotation3d(0.0, Units.degreesToRadians(-19.456239), Units.degreesToRadians(180.0 + 30.801791))
  )

  val back_right = Transform3d(
    Translation3d(Units.inchesToMeters(-8.0), Units.inchesToMeters(-11.0), Units.inchesToMeters(9.0)),
    Rotation3d(0.0, Units.degreesToRadians(-19.456239), Units.degreesToRadians(180.0 - 30.801791))
  )

  val TAG_MODEL = TargetModel(
    Units.inchesToMeters(6.5),
    Units.inchesToMeters(6.5)
  )

  /** Filtering Constants */
  const val MAX_AMBIGUITY = 0.40
  var MAX_DISTANCE_SINGLE_TAG = 3.75
  var MAX_DISTANCE_MULTI_TAG = 6.0
  val TAG_HEADING_MAX_DEV_RAD = Units.degreesToRadians(360.0)
  var MAX_HEIGHT_ERR_METERS = 0.275
  const val NUM_TAG_FACTOR = 2.0

  /** Std Dev Calculation Constants */
  const val ORDER = 2
  const val PROPORTION = 3.25

  val VISION_SIM = VisionSystemSim(
    "main"
  )

  /** Vision Sim Setup Constants */
  const val SIM_FPS = 8.0
  const val SIM_CAMERA_HEIGHT_PX = 800 // 1200 // 800
  const val SIM_CAMERA_WIDTH_PX = 1280 // 1600 // 1280
  const val SIM_FOV_DEG = 77.92 // 87.6115 // 79.09
  const val SIM_CALIB_AVG_ERR_PX = 0.45
  const val SIM_CALIB_ERR_STDDEV_PX = 0.65
  const val SIM_AVG_LATENCY = 60.0
  const val SIM_STDDEV_LATENCY = 10.0
  const val ENABLE_WIREFRAME = true

  /** List of cameras that we want to use */
  val ESTIMATORS: ArrayList<ApriltagCamera> = arrayListOf(
// Disabled for sim, reenable during merge
    ApriltagCamera(
      "reef_cam",
      TAG_LAYOUT,
      front,
      VISION_SIM
    ),
    ApriltagCamera(
      "jojocam",
      TAG_LAYOUT,
      back_left,
      VISION_SIM
    ),
    ApriltagCamera(
      "edzmjr",
      TAG_LAYOUT,
      back_right,
      VISION_SIM
    )
//    ApriltagCamera(
//      "Camera_03",
//      TAG_LAYOUT,
//      testTrans,
//      VISION_SIM
//    )

  )

  val interpolatedVisionSets: List<InterpolatedVisionDataset> = listOf(
//    InterpolatedVisionDataset.HOMEFRONT,
//    InterpolatedVisionDataset.HOMERIGHT
  )

  val ENCODER_TRUST: Matrix<N3, N1> = MatBuilder.fill(Nat.N3(), Nat.N1(), .125, .125, .0075)
  val SINGLE_TAG_TRUST: Matrix<N3, N1> = MatBuilder.fill(Nat.N3(), Nat.N1(), 0.025, 0.025, 0.01)
  val MULTI_TAG_TRUST: Matrix<N3, N1> = MatBuilder.fill(Nat.N3(), Nat.N1(), .010, .010, 0.025)
}
