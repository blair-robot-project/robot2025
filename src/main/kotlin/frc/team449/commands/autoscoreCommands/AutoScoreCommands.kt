package frc.team449.commands.autoscoreCommands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.Robot
import frc.team449.subsystems.superstructure.SuperstructureGoal

class AutoScoreCommands(private val robot: Robot) {
  private var currentCommand: Command = InstantCommand()
  private var scoreCommand: Command = InstantCommand()
  var moving = false

  fun getReefCommand(reefLocation: Pose2d, coralGoal: SuperstructureGoal.SuperstructureState): Command {
    return runOnce({
      moving = true
      val superstructureMoveCommand = robot.superstructureManager.requestGoal(coralGoal).andThen(robot.intake.holdCoral())
      scoreCommand = robot.intake.outtakeCoral()
      robot.poseSubsystem.autoscoreCommandPose = reefLocation

      currentCommand =
        AutoScoreWrapperCommand(
          robot,
          AutoScorePathfinder(robot, reefLocation, true),
          superstructureMoveCommand
        ).andThen(
          InstantCommand({
            robot.drive.defaultCommand = robot.driveCommand
            moving = false
          })
        )
      currentCommand.schedule()
    })
  }

  fun getProcessorCommand(): Command {
    return runOnce({
      moving = true
      val processorPose = if (DriverStation.getAlliance().get() == Alliance.Red) {
        AutoScoreCommandConstants.processorPoseRed
      } else {
        AutoScoreCommandConstants.processorPoseBlue
      }
      scoreCommand = InstantCommand()
      val premoveCommand = InstantCommand()
      robot.poseSubsystem.autoscoreCommandPose = processorPose
      currentCommand =
        AutoScoreWrapperCommand(
          robot,
          AutoScorePathfinder(robot, processorPose, false),
          premoveCommand
        ).andThen(
          InstantCommand({
            robot.drive.defaultCommand = robot.driveCommand
            moving = false
          })
        )
      currentCommand.schedule()
    })
  }

  fun getNetCommand(atRedSide: Boolean): Command {
    return runOnce({
      moving = true
      val netPose = Pose2d(
        AutoScoreCommandConstants.centerOfField + AutoScoreCommandConstants.centerOfField *
          if (atRedSide) 1 else -1,
        robot.poseSubsystem.pose.translation.y,
        if (atRedSide) {
          AutoScoreCommandConstants.netRotation2dRed
        } else {
          AutoScoreCommandConstants.netRotation2dBlue
        }
      )
      scoreCommand = InstantCommand()
      val premoveCommand = InstantCommand()
      currentCommand =
        AutoScoreWrapperCommand(
          robot,
          AutoScorePathfinder(robot, netPose, false),
          premoveCommand
        ).andThen(
          InstantCommand({
            robot.drive.defaultCommand = robot.driveCommand
            moving = false
          })
        )
      currentCommand.schedule()
    })
  }

  fun cancelCommand(): Command {
    return InstantCommand({
      moving = false
      currentCommand.cancel()
      robot.drive.defaultCommand = robot.driveCommand
    })
  }
}
