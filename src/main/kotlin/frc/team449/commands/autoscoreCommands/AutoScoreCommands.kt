package frc.team449.commands.autoscoreCommands

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.team449.subsystems.drive.swerve.SwerveDrive

class AutoScoreCommands (
  private val drive: SwerveDrive,
  private val desiredPosition: Translation2d
) {
  init {

  }
  private val netTranslation2d = Translation2d()
  fun netCommand() {
    drive.desiredSpeeds = ChassisSpeeds();
  }
}