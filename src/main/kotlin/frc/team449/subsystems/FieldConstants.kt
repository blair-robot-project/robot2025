package frc.team449.subsystems

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation.Alliance
import kotlin.math.PI

object FieldConstants {
  const val fieldLength = 17.55
  const val fieldWidth = 8.05

  val REEF_LOCATIONS = arrayListOf<Pose2d>()

  fun configureReef(alliance: Alliance) {
    val allianceComp = alliance == Alliance.Red

    val REEF_A = findPose(3.192615509033203, 4.189684867858887, 0.0, allianceComp)
    val REEF_B = findPose(3.192615509033203, 3.8614695072174072, 0.0, allianceComp)
    val REEF_C = findPose(3.695124626159668, 2.985105037689209, PI / 3, allianceComp)
    val REEF_D = findPose(3.9832611083984375, 2.820899248123169, PI / 3, allianceComp)
    val REEF_E = findPose(4.9979729652404785, 2.8225479125976562, 2 * PI / 3, allianceComp)
    val REEF_F = findPose(5.282362937927246, 2.989065647125244, 2 * PI / 3, allianceComp)
    val REEF_G = findPose(5.78605842590332, 3.860325813293457, PI, allianceComp)
    val REEF_H = findPose(5.78605842590332, 4.188675880432129, PI, allianceComp)
    val REEF_I = findPose(5.282362937927246, 5.065289497375488, -2 * PI / 3, allianceComp)
    val REEF_J = findPose(4.9979729652404785, 5.229397296905518, -2 * PI / 3, allianceComp)
    val REEF_K = findPose(3.9832611083984375, 5.231619358062744, -PI / 3, allianceComp)
    val REEF_L = findPose(3.695124626159668, 5.066085338592529, -PI / 3, allianceComp)

    REEF_LOCATIONS.addAll(
      listOf(
        REEF_A,
        REEF_B,
        REEF_C,
        REEF_D,
        REEF_E,
        REEF_F,
        REEF_G,
        REEF_H,
        REEF_I,
        REEF_J,
        REEF_K,
        REEF_L
      )
    )
  }

  private fun findPose(x: Double, y: Double, angle: Double, isRed: Boolean): Pose2d {
    return if (isRed) {
      Pose2d(fieldLength - x, fieldWidth - y, Rotation2d(MathUtil.angleModulus(angle + PI)))
    } else {
      Pose2d(x, y, Rotation2d(angle))
    }
  }
}
