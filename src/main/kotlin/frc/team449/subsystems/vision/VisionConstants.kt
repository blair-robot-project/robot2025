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
      AprilTag(11, Pose3d(
        0.0, 0.0, 1.0, Rotation3d()
      ))
    ),
    16.4846,
    8.1026
  )

  val TAG_LAYOUT: AprilTagFieldLayout = TEST_TAG_LAYOUT

  /** WPILib's AprilTagFieldLayout for the 2025 Reefscape Game */
//  val TAG_LAYOUT: AprilTagFieldLayout = AprilTagFieldLayout(Filesystem.getDeployDirectory().absolutePath + "/reef_only_2025.json")

  /** Robot to Camera distance */
  val front = Transform3d(
    Translation3d(Units.inchesToMeters(0.0), Units.inchesToMeters(-13.25), Units.inchesToMeters(6.9)),
    Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(-10.0), Units.degreesToRadians(-45.0))
  )

  val TAG_MODEL = TargetModel(
    Units.inchesToMeters(6.5),
    Units.inchesToMeters(6.5)
  )

  /** Filtering Constants */
  const val MAX_AMBIGUITY = 0.25
  var MAX_DISTANCE_SINGLE_TAG = 5.0
  var MAX_DISTANCE_MULTI_TAG = 6.0
  val TAG_HEADING_MAX_DEV_RAD = Units.degreesToRadians(360.0)
  var MAX_HEIGHT_ERR_METERS = 0.25
  const val NUM_TAG_FACTOR = 2.0

  /** Std Dev Calculation Constants */
  const val ORDER = 2
  const val PROPORTION = 3.25

  val VISION_SIM = VisionSystemSim(
    "main"
  )

  /** Vision Sim Setup Constants */
  const val SIM_FPS = 25.0
  const val SIM_CAMERA_HEIGHT_PX = 800 // 1200 // 800
  const val SIM_CAMERA_WIDTH_PX = 1280 // 1600 // 1280
  const val SIM_FOV_DEG = 79.09 // 87.6115 // 79.09
  const val SIM_CALIB_AVG_ERR_PX = 0.45
  const val SIM_CALIB_ERR_STDDEV_PX = 0.65
  const val SIM_AVG_LATENCY = 60.0
  const val SIM_STDDEV_LATENCY = 10.0
  const val ENABLE_WIREFRAME = true

  /** List of cameras that we want to use */
  val ESTIMATORS: ArrayList<ApriltagCamera> = arrayListOf(
    ApriltagCamera(
      "Spinel_02",
      TAG_LAYOUT,
      front,
      VISION_SIM
    )
  )

  val interpolatedVisionSets: List<InterpolatedVisionDataset> = listOf(
    InterpolatedVisionDataset.HOMEFRONT,
//    InterpolatedVisionDataset.HOMERIGHT
  )

  val ENCODER_TRUST: Matrix<N3, N1> = MatBuilder.fill(Nat.N3(), Nat.N1(), .125, .125, .0075)
  val SINGLE_TAG_TRUST: Matrix<N3, N1> = MatBuilder.fill(Nat.N3(), Nat.N1(), 0.025, 0.025, 0.35)
  val MULTI_TAG_TRUST: Matrix<N3, N1> = MatBuilder.fill(Nat.N3(), Nat.N1(), .010, .010, 0.10)
}
