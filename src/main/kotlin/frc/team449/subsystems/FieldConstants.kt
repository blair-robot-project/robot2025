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
  val REEF_LOCATIONS_PIVOT = arrayListOf<Pose2d>()
  val REEF_CENTER_LOCATIONS = arrayListOf<Pose2d>()
  const val DIST_FOR_SAFE_GI = 0.4
  var FIELD_CONFIGURED = false

  enum class ReefSide {
    LEFT,
    RIGHT
  }

  fun configureReef(alliance: Alliance) {
    FIELD_CONFIGURED = true
    val allianceComp = alliance == Alliance.Red

    val REEF_A = if (!allianceComp) {
      Pose2d(3.681, 4.020, Rotation2d(0.0)) // blue
    } else {
      Pose2d(14.327, 3.893, Rotation2d(PI)) // red
    }

    val REEF_B = if (!allianceComp) {
      Pose2d(3.68, 4.02, Rotation2d(0.0))
    } else {
      Pose2d(14.346, 4.192, Rotation2d(PI))
    }

    val REEF_C = if (!allianceComp) {
      Pose2d(3.727, 2.996, Rotation2d(PI / 3))
    } else {
      Pose2d(13.812, 5.046, Rotation2d(-2 * PI / 3))
    }

    val REEF_D = if (!allianceComp) {
      Pose2d(4.105, 3.346, Rotation2d(PI / 3))
    } else {
      Pose2d(13.560, 5.211, Rotation2d(-2 * PI / 3))
    }

    val REEF_E = if (!allianceComp) {
      Pose2d(4.892, 3.322, Rotation2d(2 * PI / 3))
    } else {
      Pose2d(12.546, 5.181, Rotation2d(-PI / 3))
    }

    val REEF_F = if (!allianceComp) {
      Pose2d(4.893, 3.317, Rotation2d(2 * PI / 3))
    } else {
      Pose2d(12.267, 5.049, Rotation2d(-PI / 3))
    }

    val REEF_G = if (!allianceComp) {
      Pose2d(5.296, 4.020, Rotation2d(PI))
    } else {
      Pose2d(11.794, 4.154, Rotation2d())
    }

    val REEF_H = if (!allianceComp) {
      Pose2d(5.300, 4.020, Rotation2d(PI))
    } else {
      Pose2d(11.771, 3.837, Rotation2d())
    }

    val REEF_I = if (!allianceComp) {
      Pose2d(5.221, 5.072, Rotation2d(-2 * PI / 3))
    } else {
      Pose2d(12.298, 2.998, Rotation2d(PI / 3))
    }

    val REEF_J = if (!allianceComp) {
      Pose2d(3.196, 4.864, Rotation2d(-2 * PI / 3))
    } else {
      Pose2d(12.655, 2.830, Rotation2d(PI / 3))
    }

    val REEF_K = if (!allianceComp) {
      Pose2d(4.086, 4.719, Rotation2d(-PI / 3)) // blue
    } else {
      Pose2d(13.572, 2.856, Rotation2d(2 * PI / 3))
    }

    val REEF_L = if (!allianceComp) {
      Pose2d(4.086, 4.725, Rotation2d(-PI / 3))
    } else {
      Pose2d(13.848, 2.994, Rotation2d(2 * PI / 3))
    }

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

    val REEF_A_PIVOT = if (!allianceComp) {
      Pose2d(3.267, 4.179, Rotation2d(PI)) // blue
    } else {
      Pose2d(14.283, 3.850, Rotation2d(0.0)) // red
    }

    val REEF_B_PIVOT = if (!allianceComp) {
      Pose2d(3.269, 3.838, Rotation2d(PI))
    } else {
      Pose2d(14.277, 4.202, Rotation2d(0.0))
    }

    val REEF_C_PIVOT = if (!allianceComp) {
      Pose2d(3.723, 3.381, Rotation2d(-2 * PI / 3))
    } else {
      Pose2d(13.825, 5.003, Rotation2d(PI / 3))
    }

    val REEF_D_PIVOT = if (!allianceComp) {
      Pose2d(4.041, 2.872, Rotation2d(-2 * PI / 3))
    } else {
      Pose2d(13.518, 5.159, Rotation2d(PI / 3))
    }

    val REEF_E_PIVOT = if (!allianceComp) {
      Pose2d(4.659, 3.03, Rotation2d(-PI / 3))
    } else {
      Pose2d(12.576, 5.150, Rotation2d(2 * PI / 3))
    }

    val REEF_F_PIVOT = if (!allianceComp) {
      Pose2d(4.659, 3.033, Rotation2d(-PI / 3))
    } else {
      Pose2d(12.283, 4.978, Rotation2d(2 * PI / 3))
    }

    val REEF_G_PIVOT = if (!allianceComp) {
      Pose2d(5.426, 3.676, Rotation2d())
    } else {
      Pose2d(11.843, 4.188, Rotation2d(PI))
    }

    val REEF_H_PIVOT = if (!allianceComp) {
      Pose2d(5.71, 4.201, Rotation2d())
    } else {
      Pose2d(11.839, 3.853, Rotation2d(PI))
    }

    val REEF_I_PIVOT = if (!allianceComp) {
      Pose2d(5.220, 5.004, Rotation2d(PI / 3))
    } else {
      Pose2d(12.319, 3.038, Rotation2d(-2 * PI / 3))
    }

    val REEF_J_PIVOT = if (!allianceComp) {
      Pose2d(4.659, 5.004, Rotation2d(PI / 3))
    } else {
      Pose2d(12.604, 2.880, Rotation2d(-2 * PI / 3))
    }

    val REEF_K_PIVOT = if (!allianceComp) {
      Pose2d(4.022, 5.1605, Rotation2d(2 * PI / 3))
    } else {
      Pose2d(13.522, 2.877, Rotation2d(-PI / 3))
    }

    val REEF_L_PIVOT = if (!allianceComp) {
      Pose2d(3.723, 4.988, Rotation2d(2 * PI / 3))
    } else {
      Pose2d(13.814, 3.048, Rotation2d(-PI / 3))
    }

    REEF_LOCATIONS_PIVOT.addAll(
      listOf(
        REEF_A_PIVOT,
        REEF_B_PIVOT,
        REEF_C_PIVOT,
        REEF_D_PIVOT,
        REEF_E_PIVOT,
        REEF_F_PIVOT,
        REEF_G_PIVOT,
        REEF_H_PIVOT,
        REEF_I_PIVOT,
        REEF_J_PIVOT,
        REEF_K_PIVOT,
        REEF_L_PIVOT
      )
    )

    val REEF_1 = findPose(3.192615509033203, (4.189684867858887 + 3.8614695072174072) / 2, 0.0, allianceComp)
    val REEF_2 = findPose((3.695124626159668 + 3.9832611083984375) / 2, (2.985105037689209 + 2.820899248123169) / 2, PI / 3, allianceComp)
    val REEF_3 = findPose((4.9979729652404785 + 5.282362937927246) / 2, (2.8225479125976562 + 2.989065647125244) / 2, 2 * PI / 3, allianceComp)
    val REEF_4 = findPose(5.78605842590332, (3.860325813293457 + 4.188675880432129) / 2, PI, allianceComp)
    val REEF_5 = findPose((4.9979729652404785 + 5.282362937927246) / 2, (5.065289497375488 + 5.229397296905518) / 2, -2 * PI / 3, allianceComp)
    val REEF_6 = findPose((3.695124626159668 + 3.9832611083984375) / 2, (5.231619358062744 + 5.066085338592529) / 2, -PI / 3, allianceComp)

    REEF_CENTER_LOCATIONS.addAll(
      arrayListOf(
        REEF_1,
        REEF_2,
        REEF_3,
        REEF_4,
        REEF_5,
        REEF_6,
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
