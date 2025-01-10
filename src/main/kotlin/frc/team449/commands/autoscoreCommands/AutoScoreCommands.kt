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


  /*
   * Takes in reefLocation enum, goes to specified reef location using swerve drive
   * Calls putCoralInReef function with passed in reefLevel enum
   * */
  fun reefCommand(reefLocation: AutoScoreCommandConstants.reefLocation,
                  reefLevel: AutoScoreCommandConstants.reefLevel) {
    when (reefLocation) {
      AutoScoreCommandConstants.reefLocation.PointA -> TODO()
      AutoScoreCommandConstants.reefLocation.PointB -> TODO()
      AutoScoreCommandConstants.reefLocation.PointC -> TODO()
      AutoScoreCommandConstants.reefLocation.PointD -> TODO()
      AutoScoreCommandConstants.reefLocation.PointE -> TODO()
      AutoScoreCommandConstants.reefLocation.PointF -> TODO()
      AutoScoreCommandConstants.reefLocation.PointG -> TODO()
      AutoScoreCommandConstants.reefLocation.PointH -> TODO()
      AutoScoreCommandConstants.reefLocation.PointI -> TODO()
      AutoScoreCommandConstants.reefLocation.PointJ -> TODO()
    }
  }
  /*
  * Puts coral in specified level
  * Should only be called by reefCommand
  * */
  fun putCoralInReef(reefLevel: AutoScoreCommandConstants.reefLevel) {
    when (reefLevel) {
      AutoScoreCommandConstants.reefLevel.L1 -> TODO()
      AutoScoreCommandConstants.reefLevel.L2 -> TODO()
      AutoScoreCommandConstants.reefLevel.L3 -> TODO()
      AutoScoreCommandConstants.reefLevel.L4 -> TODO()
    }
  }

  fun coralIntake() {

  }

  fun goToProcessor() {

  }



}