package frc.team449.commands.autoscoreCommands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.auto.choreo.PIDPoseAlign
import frc.team449.commands.autoscoreCommands.AutoScoreCommandConstants.Companion
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.vision.PoseSubsystem

class AutoScoreCommands (
  private val drive: SwerveDrive,
  private val poseSubsystem: PoseSubsystem,
) {
  
  init {

  }

  /**
   * This command moves the robot to one of the twelve reef locations 
   * (location 1 is the most vertical location on the right, going
   * clockwise)
   */
  fun moveToReefCommand(reefLocation: AutoScoreCommandConstants.reefLocation,
                  reefLevel: AutoScoreCommandConstants.reefLevel
  ) : Command {
    var reefNumericalLocation = reefLocation.ordinal + 1;
    //RANDOM POSE so that compiler does not complain about undefined when command returned.
    var reefPose = Pose2d(AutoScoreCommandConstants.reef1Translation2dRed, AutoScoreCommandConstants.reef1Rotation2dRed)

    //choose desired pose from the number and the alliance
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      when (reefNumericalLocation) {
        1 -> reefPose = AutoScoreCommandConstants.reef1PoseBlue
        2 -> reefPose = AutoScoreCommandConstants.reef2PoseBlue
        3 -> reefPose = AutoScoreCommandConstants.reef3PoseBlue
        4 -> reefPose = AutoScoreCommandConstants.reef4PoseBlue
        5 -> reefPose = AutoScoreCommandConstants.reef5PoseBlue
        6 -> reefPose = AutoScoreCommandConstants.reef6PoseBlue
        7 -> reefPose = AutoScoreCommandConstants.reef7PoseBlue
        8 -> reefPose = AutoScoreCommandConstants.reef8PoseBlue
        9 -> reefPose = AutoScoreCommandConstants.reef9PoseBlue
        10 -> reefPose = AutoScoreCommandConstants.reef10PoseBlue
        11 -> reefPose = AutoScoreCommandConstants.reef11PoseBlue
        12 -> reefPose = AutoScoreCommandConstants.reef12PoseBlue
      }
    } else /* red alliance */ {
      when (reefNumericalLocation) {
        1 -> reefPose = AutoScoreCommandConstants.reef1PoseRed
        2 -> reefPose = AutoScoreCommandConstants.reef2PoseRed
        3 -> reefPose = AutoScoreCommandConstants.reef3PoseRed
        4 -> reefPose = AutoScoreCommandConstants.reef4PoseRed
        5 -> reefPose = AutoScoreCommandConstants.reef5PoseRed
        6 -> reefPose = AutoScoreCommandConstants.reef6PoseRed
        7 -> reefPose = AutoScoreCommandConstants.reef7PoseRed
        8 -> reefPose = AutoScoreCommandConstants.reef8PoseRed
        9 -> reefPose = AutoScoreCommandConstants.reef9PoseRed
        10 -> reefPose = AutoScoreCommandConstants.reef10PoseRed
        11 -> reefPose = AutoScoreCommandConstants.reef11PoseRed
        12 -> reefPose = AutoScoreCommandConstants.reef12PoseRed
      }
    }
    
    return PIDPoseAlign(drive, poseSubsystem, reefPose)
  }
  /**
   * this command scores the coral on the reef
   * level that is passed in.
   * @param reefLevel an enum that determines which level to score the coral on
   */
  fun putCoralInReef(reefLevel: AutoScoreCommandConstants.reefLevel) : Command {
    //we don't have score yet, but we're setting up stuff for future
    //we won't have to account for alliance here
    when (reefLevel) {
      AutoScoreCommandConstants.reefLevel.L1 -> TODO()
      AutoScoreCommandConstants.reefLevel.L2 -> TODO()
      AutoScoreCommandConstants.reefLevel.L3 -> TODO()
      AutoScoreCommandConstants.reefLevel.L4 -> TODO()
    }
  }
  /**
   * moves robot to processor location using 
   * swerve drive.
   */
  fun moveToProcessorCommand() : Command {
    var returnCommand = PIDPoseAlign(drive, poseSubsystem, AutoScoreCommandConstants.processorPoseBlue)
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      returnCommand = PIDPoseAlign(drive, poseSubsystem, AutoScoreCommandConstants.processorPoseRed)
    }
    return returnCommand
  }

  /**
   * moves robot to coral intake location
   * using swerve drive.
   */
  fun moveToCoralIntakeCommand() : Command {
    var returnCommand = PIDPoseAlign(drive, poseSubsystem, AutoScoreCommandConstants.coralIntakePoseBlue)
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      returnCommand = PIDPoseAlign(drive, poseSubsystem, AutoScoreCommandConstants.coralIntakePoseRed)
    }
    return returnCommand
  }

  /**
   * moves robot to net location (only
   * distance away from the net, parallel
   * distance from net will be determined 
   * by the driver) using swerve drive
   */
  fun moveToNetCommand() : Command {
    var netPose = Pose2d(Translation2d(poseSubsystem.pose.x, AutoScoreCommandConstants.netTranslationDistance), AutoScoreCommandConstants.netRotation2dBlue)
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      netPose = Pose2d(Translation2d(poseSubsystem.pose.x, AutoScoreCommandConstants.netTranslationDistance), AutoScoreCommandConstants.netRotation2dRed)
    }
    return PIDPoseAlign(drive, poseSubsystem, netPose)
  }

  /**
   * intakes coral from coral intake
   * */
  fun intakeCoralCommand() {
    //we don't have coral intake yet, just setting up.
  }



}