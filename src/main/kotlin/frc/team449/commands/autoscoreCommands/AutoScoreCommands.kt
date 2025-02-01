package frc.team449.commands.autoscoreCommands

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.Robot
import frc.team449.auto.choreo.MagnetizePIDPoseAlign
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.superstructure.SuperstructureGoal
import frc.team449.subsystems.vision.PoseSubsystem

class AutoScoreCommands(
  private val drive: SwerveDrive,
  private val poseSubsystem: PoseSubsystem,
  private val controller: XboxController,
  private val robot: Robot
) {

  val constraints = PathConstraints(
    30.0,
    40.0,
    Units.degreesToRadians(540.0),
    Units.degreesToRadians(720.0)
  )

  val usingPathfinding = true

  init {
    if (usingPathfinding) println("pathfinding") else println("magnezite")
  }

  /**
   * This command moves the robot to one of the twelve reef locations
   * (location 1 is the most vertical location on the right, going
   * clockwise)
   * @param reefLocation a reefLocationEnum that defines which spot to go to, defined with the numeric system above.
   */

  fun moveToReefCommand(
    reefLocation: AutoScoreCommandConstants.ReefLocation
  ): Command {
    println("reef thing pressed")
    var reefNumericalLocation = reefLocation.ordinal + 1
    // RANDOM POSE so that compiler does not complain about undefined when command returned.
    // var reefPose = Pose2d(AutoScoreCommandConstants.reef1Translation2dRed, AutoScoreCommandConstants.reef1Rotation2dRed)
    var reefPose = AutoScoreCommandConstants.reef1PoseRed
    // choose desired pose from the number and the alliance
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
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
    if (usingPathfinding) {
      /*** pathfinding ***/
      return AutoBuilder.pathfindToPose(
        reefPose,
        constraints,
        0.0,
        // Goal end velocity in meters/sec
      )
    } else {
      /*** magnetize ***/
      return MagnetizePIDPoseAlign(drive, poseSubsystem, reefPose, controller)
    }
  }

  /**
   * moves robot to processor location using
   * swerve drive.
   */
  fun moveToProcessorCommand(): Command {
    var pose = AutoScoreCommandConstants.processorPoseBlue

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      pose = AutoScoreCommandConstants.processorPoseRed
    }

    var returnCommand = AutoBuilder.pathfindToPose(
      pose,
      constraints,
      0.0,
    )

    if(!usingPathfinding) {
      returnCommand = MagnetizePIDPoseAlign(
        drive,
        poseSubsystem,
        pose,
        controller
      )
    }
    return returnCommand
  }

  /**
   * moves robot to coral intake, either top
   * or bottom depending on the parameter passed
   * in, using swerve drive
   * @param isAtTopSource a boolean representing if we're intaking from the top or the bottom source. True if top, false if bottom.
   */
  fun moveToCoralIntakeCommand(isAtTopSource: Boolean): Command {
    var pose = AutoScoreCommandConstants.coralIntakePoseBlueBottom
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      if(isAtTopSource) {
        pose = AutoScoreCommandConstants.coralIntakePoseRedTop
      } else {
        pose = AutoScoreCommandConstants.coralIntakePoseRedBottom
      }
    } else {
      if(isAtTopSource) {
        pose = AutoScoreCommandConstants.coralIntakePoseBlueTop
      } else {
        pose =
          AutoScoreCommandConstants.coralIntakePoseBlueBottom
      }
    }
    var returnCommand = AutoBuilder.pathfindToPose(
      pose,
      constraints,
      0.0,
    )
    if (!usingPathfinding) {
      returnCommand = MagnetizePIDPoseAlign(
        drive,
        poseSubsystem,
        pose,
        controller
      )
    }
    return returnCommand
  }

  /**
   * returns a command that moves robot to
   * net location on either side (only
   * distance away from the net,
   * parallel distance from net will be
   * determined by the driver) using swerve
   * drive.
   * @param onRedAllianceSide a boolean representing which side of the field we're on. If true, the robot moves to the red alliance side to score net.
   */
  fun moveToNetCommand(onRedAllianceSide: Boolean): Command {
    var pose = Pose2d(Translation2d(AutoScoreCommandConstants.centerOfField - AutoScoreCommandConstants.netTranslationDistance, poseSubsystem.pose.y), AutoScoreCommandConstants.netRotation2dBlue)
    if (onRedAllianceSide) {
      pose = Pose2d(Translation2d(AutoScoreCommandConstants.centerOfField + AutoScoreCommandConstants.netTranslationDistance, poseSubsystem.pose.y), AutoScoreCommandConstants.netRotation2dRed)
    }

    if (usingPathfinding) {
      /*** pathfinding ***/
      val pathfindingCommand = AutoBuilder.pathfindToPose(
        pose,
        constraints,
        10.0,
        // Goal end velocity in meters/sec
      )
      return pathfindingCommand
    } else {
      /*** magnetize ***/
    return MagnetizePIDPoseAlign(drive, poseSubsystem, pose, controller)
    }
  }

  /**
   * returns a command scores the coral on the reef
   * level that is passed in.
   * @param reefLevel a reefLevel enum that determines which level to score the coral on
   * does nothing right now
   */
  fun putCoralInReef(reefLevel: AutoScoreCommandConstants.ReefLevel): Command {
    return when (reefLevel) {
      AutoScoreCommandConstants.ReefLevel.L1 -> robot.superstructureManager.requestGoal(SuperstructureGoal.L1)
      AutoScoreCommandConstants.ReefLevel.L2 -> robot.superstructureManager.requestGoal(SuperstructureGoal.L2)
      AutoScoreCommandConstants.ReefLevel.L3 -> robot.superstructureManager.requestGoal(SuperstructureGoal.L3)
      AutoScoreCommandConstants.ReefLevel.L4 -> robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
    }
    //premove will be added in the bindings instead of here
  }

  /**
   * returns a command that scores into processor
   * does nothing right now
   * */
  fun scoreProcessorCommand(): Command {
    // we don't have processor scoring yet, just setting up.
    // returns a command that does nothing (for now)
    return InstantCommand()
  }

  /**
   * returns a command that scores net into net
   * does nothing right now
   * */
  fun scoreNetCommand(): Command {
    // we don't have net scoring yet, just setting up.
    // returns a command that does nothing (for now)
    return InstantCommand()
  }

  /**
   * intakes coral from coral station
   * */
  fun intakeCoralCommand(): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
  }
}