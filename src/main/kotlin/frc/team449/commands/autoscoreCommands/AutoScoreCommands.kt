package frc.team449.commands.autoscoreCommands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
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
   * @param reefLocation a reefLocationEnum that defines which spot to go to, defined with the numeric system above.
   */
  fun moveToReefCommand(reefLocation: AutoScoreCommandConstants.reefLocation,
  ) : Command {
    var reefNumericalLocation = reefLocation.ordinal + 1;
    //RANDOM POSE so that compiler does not complain about undefined when command returned.
    var reefPose = Pose2d(AutoScoreCommandConstants.reef1Translation2dRed, AutoScoreCommandConstants.reef1Rotation2dRed)

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
      13 -> reefPose = AutoScoreCommandConstants.reef1PoseRed
      14 -> reefPose = AutoScoreCommandConstants.reef2PoseRed
      15 -> reefPose = AutoScoreCommandConstants.reef3PoseRed
      16 -> reefPose = AutoScoreCommandConstants.reef4PoseRed
      17 -> reefPose = AutoScoreCommandConstants.reef5PoseRed
      18 -> reefPose = AutoScoreCommandConstants.reef6PoseRed
      19 -> reefPose = AutoScoreCommandConstants.reef7PoseRed
      20 -> reefPose = AutoScoreCommandConstants.reef8PoseRed
      21 -> reefPose = AutoScoreCommandConstants.reef9PoseRed
      22 -> reefPose = AutoScoreCommandConstants.reef10PoseRed
      23 -> reefPose = AutoScoreCommandConstants.reef11PoseRed
      24 -> reefPose = AutoScoreCommandConstants.reef12PoseRed
    }

    
    return PIDPoseAlign(drive, poseSubsystem, reefPose)
  }
  /**
   * this command scores the coral on the reef
   * level that is passed in.
   * @param reefLevel a reefLevel enum that determines which level to score the coral on
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
  fun moveToProcessorCommandBlue() : Command {
    var returnCommand = PIDPoseAlign(drive, poseSubsystem, AutoScoreCommandConstants.processorPoseBlue)
    returnCommand = PIDPoseAlign(drive, poseSubsystem, AutoScoreCommandConstants.processorPoseBlue)
    return returnCommand
  }
  fun moveToProcessorCommandRed() : Command {
    var returnCommand = PIDPoseAlign(drive, poseSubsystem, AutoScoreCommandConstants.processorPoseRed)
    returnCommand = PIDPoseAlign(drive, poseSubsystem, AutoScoreCommandConstants.processorPoseRed)
    return returnCommand
  }

  /**
   * moves robot to coral intake, either top
   * or bottom depending on the parameter passed
   * in, using swerve drive
   * @param isAtTopSource a boolean representing if we're intaking from the top or the bottom source. True if top, false if bottom.
   */

  fun moveToCoralIntakeCommandBlue(isAtTopSource : Boolean) : Command {
    var returnCommand = PIDPoseAlign(drive, poseSubsystem, AutoScoreCommandConstants.coralIntakePoseBlueTop)
    if(!isAtTopSource) {
      returnCommand = PIDPoseAlign(drive, poseSubsystem, AutoScoreCommandConstants.coralIntakePoseBlueBottom)
    }
    return returnCommand
  }
  fun moveToCoralIntakeCommandRed(isAtTopSource : Boolean) : Command {
    var returnCommand = PIDPoseAlign(drive, poseSubsystem, AutoScoreCommandConstants.coralIntakePoseRedTop)
    if(!isAtTopSource) {
      returnCommand = PIDPoseAlign(drive, poseSubsystem, AutoScoreCommandConstants.coralIntakePoseRedBottom)
    }
    return returnCommand
  }

  /**
   * moves robot to net location on either
   * side (only distance away from the net,
   * parallel distance from net will be
   * determined by the driver) using swerve
   * drive.
   * @param onRedAllianceSide a boolean representing which side of the field we're on. If true, the robot moves to the red alliance side to score net.
   */
  fun moveToNetCommand(onRedAllianceSide: Boolean) : Command {
    var netPose = Pose2d(Translation2d(AutoScoreCommandConstants.centerOfField - AutoScoreCommandConstants.netTranslationDistance, poseSubsystem.pose.y), AutoScoreCommandConstants.netRotation2dBlue)
    if(onRedAllianceSide) {
      netPose = Pose2d(Translation2d(AutoScoreCommandConstants.centerOfField + AutoScoreCommandConstants.netTranslationDistance, poseSubsystem.pose.y), AutoScoreCommandConstants.netRotation2dBlue)
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