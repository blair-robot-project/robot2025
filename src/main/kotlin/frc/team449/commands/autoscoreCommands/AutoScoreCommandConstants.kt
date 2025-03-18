package frc.team449.commands.autoscoreCommands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import frc.team449.subsystems.drive.swerve.SwerveConstants
import kotlin.math.PI

class AutoScoreCommandConstants {

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

    // pathfinding vars
    val MAX_ACCEL = 4.0 // m/s/s
    val MAX_ROT_ACCEL = 2 * PI // rad/s/s
    val MAX_PATHFINDING_ROT_SPEED = 2 * PI // rad/s
    val MAX_PATHFINDING_LINEAR_SPEED = SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED * 7 / 10 // m/s

    // pathmag vars
    val MAX_ROT_SPEED = 2 * PI // rad/s
    val MAX_LINEAR_SPEED = SwerveConstants.MAX_ATTAINABLE_MK4I_SPEED * 0.9 // m/s
  }
}
