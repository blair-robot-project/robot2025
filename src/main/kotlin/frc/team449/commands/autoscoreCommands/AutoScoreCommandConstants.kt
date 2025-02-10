package frc.team449.commands.autoscoreCommands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import frc.team449.subsystems.drive.swerve.SwerveConstants
import kotlin.math.PI

class AutoScoreCommandConstants() {

  enum class ReefLocation {
    Location1,
    Location2,
    Location3,
    Location4,
    Location5,
    Location6,
    Location7,
    Location8,
    Location9,
    Location10,
    Location11,
    Location12
  }

  enum class ReefLevel {
    L1,
    L2,
    L3,
    L4
  }

  companion object {
    /**
     * syntactic sugar method
     * */
    fun radians(degree: Int): Double {
      return Units.degreesToRadians(degree.toDouble())
    }
    // FILLER VALUES CURRENTLY

    // processor pose values
    // BLUE
    val processorTranslation2dBlue = Translation2d(6.358, 0.622)
    val processorRotation2dBlue = Rotation2d(3 * Math.PI / 2) // in radians
    val processorPoseBlue = Pose2d(processorTranslation2dBlue, processorRotation2dBlue)

    val testTranslation = Translation2d(6.358, 1.622)
    val testRotation = Rotation2d(3 * Math.PI / 2) // in radians
    val testPose = Pose2d(testTranslation, testRotation)

    // RED
    val processorTranslation2dRed = Translation2d(11.520, 7.500)
    val processorRotation2dRed = Rotation2d(Math.PI / 2) // in radians
    val processorPoseRed = Pose2d(processorTranslation2dRed, processorRotation2dRed)

    // coral intake pose values
    // BLUE
    // TOP
    val coralIntakeTranslation2dBlueTop = Translation2d(16.454, 6.990)
    val coralIntakeRotation2dBlueTop = Rotation2d(radians(55)) // in radians
    val coralIntakePoseBlueTop = Pose2d(coralIntakeTranslation2dBlueTop, coralIntakeRotation2dBlueTop)

    // BOTTOM
    val coralIntakeTranslation2dBlueBottom = Translation2d(16.454, 1.111)
    val coralIntakeRotation2dBlueBottom = Rotation2d(radians(-55)) // in radians
    val coralIntakePoseBlueBottom = Pose2d(coralIntakeTranslation2dBlueBottom, coralIntakeRotation2dBlueBottom)

    // RED
    // TOP
    val coralIntakeTranslation2dRedTop = Translation2d(1.214, 6.990)
    val coralIntakeRotation2dRedTop = Rotation2d(radians(125)) // in radians
    val coralIntakePoseRedTop = Pose2d(coralIntakeTranslation2dRedTop, coralIntakeRotation2dRedTop)

    // BOTTOM
    val coralIntakeTranslation2dRedBottom = Translation2d(1.214, 1.111)
    val coralIntakeRotation2dRedBottom = Rotation2d(radians(-125)) // in radians
    val coralIntakePoseRedBottom = Pose2d(coralIntakeTranslation2dRedBottom, coralIntakeRotation2dRedBottom)

    // net pose values
    val centerOfField = 8.808
    val netTranslationDistance: Double = 2.152

    // BLUE
    val netRotation2dBlue = Rotation2d(0.0) // in radians

    // RED
    val netRotation2dRed = Rotation2d(Math.PI) // in radians

    // Reef pose translations are in meters (round to 3 decimal places)
    // Reef pose angles are in radians
    // BLUE
    // reef 1st pose values
    val reef1Translation2dBlue = Translation2d(4.906, 5.1359)

    // RADIANS
    val reef1Rotation2dBlue = Rotation2d(radians(240))
    val reef1PoseBlue = Pose2d(reef1Translation2dBlue, reef1Rotation2dBlue)

    // reef 2nd pose values
    val reef2Translation2dBlue = Translation2d(5.239, 4.978)

    // RADIANS
    val reef2Rotation2dBlue = Rotation2d(radians(240))
    val reef2PoseBlue = Pose2d(reef2Translation2dBlue, reef2Rotation2dBlue)

    // reef 3rd pose values
    val reef3Translation2dBlue = Translation2d(5.851, 4.156)

    // RADIANS
    val reef3Rotation2dBlue = Rotation2d(radians(180))
    val reef3PoseBlue = Pose2d(reef3Translation2dBlue, reef3Rotation2dBlue)

    // reef 4th pose values
    val reef4Translation2dBlue = Translation2d(5.851, 3.876)

    // RADIANS
    val reef4Rotation2dBlue = Rotation2d(radians(180))
    val reef4PoseBlue = Pose2d(reef4Translation2dBlue, reef4Rotation2dBlue)

    // reef 5th pose values
    val reef5Translation2dBlue = Translation2d(5.291, 2.966)

    // RADIANS
    val reef5Rotation2dBlue = Rotation2d(radians(120))
    val reef5PoseBlue = Pose2d(reef5Translation2dBlue, reef5Rotation2dBlue)

    // reef 6th pose values
    val reef6Translation2dBlue = Translation2d(5.011, 2.809)

    // RADIANS
    val reef6Rotation2dBlue = Rotation2d(radians(120))
    val reef6PoseBlue = Pose2d(reef6Translation2dBlue, reef6Rotation2dBlue)

    // reef 7th pose values
    val reef7Translation2dBlue = Translation2d(3.979, 2.809)

    // RADIANS
    val reef7Rotation2dBlue = Rotation2d(radians(60))
    val reef7PoseBlue = Pose2d(reef7Translation2dBlue, reef7Rotation2dBlue)

    // reef 8th pose values
    val reef8Translation2dBlue = Translation2d(3.700, 2.966)

    // RADIANS
    val reef8Rotation2dBlue = Rotation2d(radians(60))
    val reef8PoseBlue = Pose2d(reef8Translation2dBlue, reef8Rotation2dBlue)

    // reef 9th pose values
    val reef9Translation2dBlue = Translation2d(3.174, 3.876)

    // RADIANS
    val reef9Rotation2dBlue = Rotation2d((radians(0)))
    val reef9PoseBlue = Pose2d(reef9Translation2dBlue, reef9Rotation2dBlue)

    // reef 10th pose values
    val reef10Translation2dBlue = Translation2d(3.174, 4.173)

    // RADIANS
    val reef10Rotation2dBlue = Rotation2d(radians(0))
    val reef10PoseBlue = Pose2d(reef10Translation2dBlue, reef10Rotation2dBlue)

    // reef 11th pose values
    val reef11Translation2dBlue = Translation2d(3.7340, 4.9601)

    // RADIANS
    val reef11Rotation2dBlue = Rotation2d(radians(-60))

    val reef11PoseBlue = Pose2d(reef11Translation2dBlue, reef11Rotation2dBlue)

    // reef 12th pose values
    val reef12Translation2dBlue = Translation2d(4.0490, 5.153)

    // RADIANS
    val reef12Rotation2dBlue = Rotation2d(radians(-60))
    val reef12PoseBlue = Pose2d(reef12Translation2dBlue, reef12Rotation2dBlue)

    // RED
    // reef 1st pose values
    val reef1Translation2dRed = Translation2d(13.549, 5.136)

    // RADIANS
    val reef1Rotation2dRed = Rotation2d(radians(240))
    val reef1PoseRed = Pose2d(reef1Translation2dRed, reef1Rotation2dRed)

    // reef 2nd pose values
    val reef2Translation2dRed = Translation2d(13.794, 4.943)

    // RADIANS
    val reef2Rotation2dRed = Rotation2d(radians(240))
    val reef2PoseRed = Pose2d(reef2Translation2dRed, reef2Rotation2dRed)

    // reef 3rd pose values
    val reef3Translation2dRed = Translation2d(14.301, 4.121)

    // RADIANS
    val reef3Rotation2dRed = Rotation2d(radians(180))
    val reef3PoseRed = Pose2d(reef3Translation2dRed, reef3Rotation2dRed)

    // reef 4th pose values
    val reef4Translation2dRed = Translation2d(14.267, 3.859)

    // RADIANS
    val reef4Rotation2dRed = Rotation2d(radians(180))
    val reef4PoseRed = Pose2d(reef4Translation2dRed, reef4Rotation2dRed)

    // reef 5th pose values
    val reef5Translation2dRed = Translation2d(13.864, 2.966)

    // RADIANS
    val reef5Rotation2dRed = Rotation2d(radians(120))
    val reef5PoseRed = Pose2d(reef5Translation2dRed, reef5Rotation2dRed)

    // reef 6th pose values
    val reef6Translation2dRed = Translation2d(13.601, 2.809)

    // RADIANS
    val reef6Rotation2dRed = Rotation2d(radians(120))
    val reef6PoseRed = Pose2d(reef6Translation2dRed, reef6Rotation2dRed)

    // reef 7th pose values
    val reef7Translation2dRed = Translation2d(12.534, 2.809)

    // RADIANS
    val reef7Rotation2dRed = Rotation2d(radians(60))
    val reef7PoseRed = Pose2d(reef7Translation2dRed, reef7Rotation2dRed)

    // reef 8th pose values
    val reef8Translation2dRed = Translation2d(12.237, 2.966)

    // RADIANS
    val reef8Rotation2dRed = Rotation2d(radians(60))
    val reef8PoseRed = Pose2d(reef8Translation2dRed, reef8Rotation2dRed)

    // reef 9th pose values
    val reef9Translation2dRed = Translation2d(11.730, 3.876)

    // RADIANS
    val reef9Rotation2dRed = Rotation2d((radians(0)))
    val reef9PoseRed = Pose2d(reef9Translation2dRed, reef9Rotation2dRed)

    // reef 10th pose values
    val reef10Translation2dRed = Translation2d(11.730, 4.173)

    // RADIANS
    val reef10Rotation2dRed = Rotation2d(radians(0))
    val reef10PoseRed = Pose2d(reef10Translation2dRed, reef10Rotation2dRed)

    // reef 11th pose values
    val reef11Translation2dRed = Translation2d(12.307, 4.995)

    // RADIANS
    val reef11Rotation2dRed = Rotation2d(radians(-60))
    val reef11PoseRed = Pose2d(reef11Translation2dRed, reef11Rotation2dRed)

    // reef 12th pose values
    val reef12Translation2dRed = Translation2d(12.604, 5.171)

    // RADIANS
    val reef12Rotation2dRed = Rotation2d(radians(-60))
    val reef12PoseRed = Pose2d(reef12Translation2dRed, reef12Rotation2dRed)

    const val ROT_MAX_ACCEL = 2 * PI
    const val MAX_ACCEL = 10.0
    const val MAX_ROT_SPEED = 5 * PI / 4 // r ad/s
    val MAX_LINEAR_SPEED = SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED // m/s
  }
}
