package frc.team449.commands.autoscoreCommands

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.team449.Robot
import frc.team449.commands.driveAlign.SimpleReefAlign
import frc.team449.subsystems.superstructure.SuperstructureGoal

class AutoScoreCommands(private val robot : Robot) {
  fun getReefCommand(rl : AutoScoreCommandConstants.ReefLocation, cl : AutoScoreCommandConstants.CoralLevel) : Command {
    val reefLocationPose =
      if(DriverStation.getAlliance().get() == Alliance.Red) {
        when(rl) {
          AutoScoreCommandConstants.ReefLocation.Location1 -> AutoScoreCommandConstants.reef1PoseRed
          AutoScoreCommandConstants.ReefLocation.Location2 -> AutoScoreCommandConstants.reef2PoseRed
          AutoScoreCommandConstants.ReefLocation.Location3 -> AutoScoreCommandConstants.reef3PoseRed
          AutoScoreCommandConstants.ReefLocation.Location4 -> AutoScoreCommandConstants.reef4PoseRed
          AutoScoreCommandConstants.ReefLocation.Location5 -> AutoScoreCommandConstants.reef5PoseRed
          AutoScoreCommandConstants.ReefLocation.Location6 -> AutoScoreCommandConstants.reef6PoseRed
          AutoScoreCommandConstants.ReefLocation.Location7 -> AutoScoreCommandConstants.reef7PoseRed
          AutoScoreCommandConstants.ReefLocation.Location8 -> AutoScoreCommandConstants.reef8PoseRed
          AutoScoreCommandConstants.ReefLocation.Location9 -> AutoScoreCommandConstants.reef9PoseRed
          AutoScoreCommandConstants.ReefLocation.Location10 -> AutoScoreCommandConstants.reef10PoseRed
          AutoScoreCommandConstants.ReefLocation.Location11 -> AutoScoreCommandConstants.reef11PoseRed
          AutoScoreCommandConstants.ReefLocation.Location12 -> AutoScoreCommandConstants.reef12PoseRed
        }
      } else {
        when(rl) {
          AutoScoreCommandConstants.ReefLocation.Location1 -> AutoScoreCommandConstants.reef1PoseBlue
          AutoScoreCommandConstants.ReefLocation.Location2 -> AutoScoreCommandConstants.reef2PoseBlue
          AutoScoreCommandConstants.ReefLocation.Location3 -> AutoScoreCommandConstants.reef3PoseBlue
          AutoScoreCommandConstants.ReefLocation.Location4 -> AutoScoreCommandConstants.reef4PoseBlue
          AutoScoreCommandConstants.ReefLocation.Location5 -> AutoScoreCommandConstants.reef5PoseBlue
          AutoScoreCommandConstants.ReefLocation.Location6 -> AutoScoreCommandConstants.reef6PoseBlue
          AutoScoreCommandConstants.ReefLocation.Location7 -> AutoScoreCommandConstants.reef7PoseBlue
          AutoScoreCommandConstants.ReefLocation.Location8 -> AutoScoreCommandConstants.reef8PoseBlue
          AutoScoreCommandConstants.ReefLocation.Location9 -> AutoScoreCommandConstants.reef9PoseBlue
          AutoScoreCommandConstants.ReefLocation.Location10 -> AutoScoreCommandConstants.reef10PoseBlue
          AutoScoreCommandConstants.ReefLocation.Location11 -> AutoScoreCommandConstants.reef11PoseBlue
          AutoScoreCommandConstants.ReefLocation.Location12 -> AutoScoreCommandConstants.reef12PoseBlue
        }
      }
    val premoveGoal = when(cl) {
      AutoScoreCommandConstants.CoralLevel.L1 -> SuperstructureGoal.L1_PREMOVE
      AutoScoreCommandConstants.CoralLevel.L2 -> SuperstructureGoal.L2_PREMOVE
      AutoScoreCommandConstants.CoralLevel.L3 -> SuperstructureGoal.L3_PREMOVE
      AutoScoreCommandConstants.CoralLevel.L4 -> SuperstructureGoal.L4_PREMOVE
    }
    val scoreGoal = when(cl) {
      AutoScoreCommandConstants.CoralLevel.L1 -> SuperstructureGoal.L1
      AutoScoreCommandConstants.CoralLevel.L2 -> SuperstructureGoal.L2
      AutoScoreCommandConstants.CoralLevel.L3 -> SuperstructureGoal.L3
      AutoScoreCommandConstants.CoralLevel.L4 -> SuperstructureGoal.L4
    }
    robot.poseSubsystem.autoscoreCommandPose = reefLocationPose
    return AutoscoreWrapperCommand(robot, AutoScorePathfinder(robot, reefLocationPose), premoveGoal, reefLocationPose)
      .andThen(PrintCommand("rotation done"))
      //.andThen(SimpleReefAlign(robot.drive, robot.poseSubsystem))
      .andThen(PrintCommand("movement done"))
//      .andThen(robot.superstructureManager.requestGoal(scoreGoal))
      .andThen(runOnce({ robot.drive.defaultCommand = robot.driveCommand }))
      .andThen(PrintCommand("autoscore command done"))
  }
}
