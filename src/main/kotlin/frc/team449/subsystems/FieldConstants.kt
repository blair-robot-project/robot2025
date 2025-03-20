package frc.team449.subsystems

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.team449.commands.autoscoreCommands.AutoScoreCommandConstants.Companion.centerOfField
import kotlin.math.PI

object FieldConstants {
  const val fieldLength = 17.55
  const val fieldWidth = 8.05

  val REEF_LOCATIONS = arrayListOf<Pose2d>()
  val REEF_CENTER_LOCATIONS = arrayListOf<Pose2d>()
  lateinit var REEF_CENTER: Translation2d
  val CORAL_INTAKE_LOCATIONS = arrayListOf<Translation2d>()
  val APRIL_TAG_LOCATIONS = arrayListOf<Translation2d>()

  enum class ReefSide {
    LEFT,
    RIGHT
  }

  fun configureReef(alliance: Alliance) {
    val allianceComp = alliance == Alliance.Red

    val REEF_A = if (!allianceComp) {
      Pose2d(3.212, 4.168, Rotation2d(0.0))
    } else {
      Pose2d(14.335, 3.895, Rotation2d(PI))
    }

    val REEF_B = if (!allianceComp) {
      Pose2d(3.195, 3.856, Rotation2d(0.0))
    } else {
      Pose2d(14.335, 4.182, Rotation2d(PI))
    }

    val REEF_C = if (!allianceComp) {
      Pose2d(3.739, 2.981, Rotation2d(PI / 3))
    } else {
      Pose2d(13.826, 5.05, Rotation2d(-2 * PI / 3))
    }

    val REEF_D = if (!allianceComp) {
      Pose2d(4.0, 2.805, Rotation2d(PI / 3))
    } else {
      Pose2d(13.565, 5.225, Rotation2d(-2 * PI / 3))
    }

    val REEF_E = if (!allianceComp) {
      Pose2d(5.020, 2.858, Rotation2d(2 * PI / 3))
    } else {
      Pose2d(12.532, 5.191, Rotation2d(-PI / 3))
    }

    val REEF_F = if (!allianceComp) {
      Pose2d(5.281, 2.984, Rotation2d(2 * PI / 3))
    } else {
      Pose2d(12.261, 5.053, Rotation2d(-PI / 3))
    }

    val REEF_G = if (!allianceComp) {
      Pose2d(5.763, 3.880, Rotation2d(PI))
    } else {
      Pose2d(11.783, 4.147, Rotation2d())
    }

    val REEF_H = if (!allianceComp) {
      Pose2d(5.783, 4.191, Rotation2d(PI))
    } else {
      Pose2d(11.764, 3.860, Rotation2d())
    }

    val REEF_I = if (!allianceComp) {
      Pose2d(5.260, 5.05, Rotation2d(-2 * PI / 3))
    } else {
      Pose2d(12.301, 2.984, Rotation2d(PI / 3))
    }

    val REEF_J = if (!allianceComp) {
      Pose2d(4.984, 5.231, Rotation2d(-2 * PI / 3))
    } else {
      Pose2d(12.546, 2.819, Rotation2d(PI / 3))
    }

    val REEF_K = if (!allianceComp) {
      Pose2d(3.982, 5.199, Rotation2d(-PI / 3))
    } else {
      Pose2d(13.575, 2.850, Rotation2d(2 * PI / 3))
    }

    val REEF_L = if (!allianceComp) {
      Pose2d(3.700, 5.060, Rotation2d(-PI / 3))
    } else {
      Pose2d(13.850, 2.980, Rotation2d(2 * PI / 3))
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

    val REEF_1 = findPose(3.192615509033203, (4.189684867858887 + 3.8614695072174072) / 2, 0.0, allianceComp)
    val REEF_2 = findPose((3.695124626159668 + 3.9832611083984375) / 2, (2.985105037689209 + 2.820899248123169) / 2, PI / 3, allianceComp)
    val REEF_3 = findPose((4.9979729652404785 + 5.282362937927246) / 2, (2.8225479125976562 + 2.989065647125244) / 2, 2 * PI / 3, allianceComp)
    val REEF_4 = findPose(5.78605842590332, (3.860325813293457 + 4.188675880432129) / 2, PI, allianceComp)
    val REEF_5 = findPose((4.9979729652404785 + 5.282362937927246) / 2, (5.065289497375488 + 5.229397296905518) / 2, -2 * PI / 3, allianceComp)
    val REEF_6 = findPose((3.695124626159668 + 3.9832611083984375) / 2, (5.231619358062744 + 5.066085338592529) / 2, -PI / 3, allianceComp)

    REEF_CENTER_LOCATIONS.addAll(
      listOf(
        REEF_1,
        REEF_2,
        REEF_3,
        REEF_4,
        REEF_5,
        REEF_6,
      )
    )

    val RED_REEF_CENTER = Translation2d(13.0758, 4.0325)
    val BLUE_REEF_CENTER = Translation2d(centerOfField - (RED_REEF_CENTER.x - centerOfField), RED_REEF_CENTER.y)
    REEF_CENTER = (if (allianceComp) RED_REEF_CENTER else BLUE_REEF_CENTER)

    val CORAL_INTAKE_BLUE_TOP = Translation2d(1.4578, 7.1276)
    val CORAL_INTAKE_BLUE_BOTTOM = Translation2d(1.4578, 1.1276)

    val CORAL_INTAKE_RED_TOP = Translation2d(16.1578, 7.1276)
    val CORAL_INTAKE_RED_BOTTOM = Translation2d(16.1578, 1.1276)
    CORAL_INTAKE_LOCATIONS.addAll(
      if (allianceComp) {
        listOf(
          CORAL_INTAKE_RED_TOP,
          CORAL_INTAKE_RED_BOTTOM
        )
      } else {
        listOf(
          CORAL_INTAKE_BLUE_TOP,
          CORAL_INTAKE_BLUE_BOTTOM
        )
      }
    )
    val APRIL_TAGS_RED = listOf(
      Translation2d(12.215, 3.995),
      Translation2d(12.6488, 4.772),
      Translation2d(13.501, 4.754),
      Translation2d(13.9364, 4.006),
      Translation2d(13.518, 3.258),
      Translation2d(12.631, 3.275)
    )
    val APRIL_TAGS_BLUE = arrayListOf<Translation2d>()
    val distanceBetweenReefs = RED_REEF_CENTER.getDistance(BLUE_REEF_CENTER)
    for (translation in APRIL_TAGS_RED) {
      APRIL_TAGS_BLUE.add(Translation2d(translation.x - distanceBetweenReefs, translation.y))
    }
    APRIL_TAG_LOCATIONS.addAll(if (allianceComp) APRIL_TAGS_RED else APRIL_TAGS_BLUE)
  }

  private fun findPose(x: Double, y: Double, angle: Double, isRed: Boolean): Pose2d {
    return if (isRed) {
      Pose2d(fieldLength - x, fieldWidth - y, Rotation2d(MathUtil.angleModulus(angle + PI)))
    } else {
      Pose2d(x, y, Rotation2d(angle))
    }
  }
}
