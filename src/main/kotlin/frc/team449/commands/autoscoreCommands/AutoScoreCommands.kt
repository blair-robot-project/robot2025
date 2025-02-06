package frc.team449.commands.autoscoreCommands

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.path.PathPoint
import com.pathplanner.lib.path.RotationTarget
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.team449.Robot
import frc.team449.subsystems.superstructure.SuperstructureGoal
import frc.team449.subsystems.vision.PoseSubsystem
import frc.team449.subsystems.RobotConstants

class FollowPathCommandCooked(val poseSubsystem: PoseSubsystem, command: Command) : Command() {

  private val currentCommand = command
  private val autodistanceTime = poseSubsystem.autoDistance * 4.0
  private var autodistanceTimer = autodistanceTime

  override fun initialize() {
    currentCommand.initialize()
  }

  override fun execute() {
    currentCommand.execute()
  }

  private fun resetAndEndCommand() {
    autodistanceTimer = autodistanceTime
    currentCommand.end(true)
    currentCommand.cancel()
  }

  override fun isFinished(): Boolean {
    if (autodistanceTimer < 0) {
      println("autoscore command finished")
      resetAndEndCommand()
      return true
    }
    if (poseSubsystem.pose.translation.getDistance(poseSubsystem.autoscoreCommandPose.translation) < poseSubsystem.autoDistance) {
      autodistanceTimer -= 0.05
      // prevent command from timing out if we're close
    }
    return false
  }

  override fun end(interrupted: Boolean) {
    currentCommand.end(interrupted)
  }
}

class AutoScoreCommands(
  val poseSubsystem: PoseSubsystem,
  private val robot: Robot
) {

  private val constraints = PathConstraints(
    RobotConstants.MAX_LINEAR_SPEED,
    RobotConstants.MAX_ACCEL,
    RobotConstants.MAX_ROT_SPEED,
    Units.degreesToRadians(720.0)
  )


  lateinit var currentCommand: Command

  /**
   * This command moves the robot to one of the twelve reef locations
   * (location 1 is the most vertical location on the right, going
   * clockwise)
   * @param reefLocation a reefLocationEnum that defines which spot to go to, defined with the numeric system above.
   */

  private fun moveToReefCommand(
    reefLocation: AutoScoreCommandConstants.ReefLocation
  ): Command {
    println("reef command called")
    val reefNumericalLocation = reefLocation.ordinal + 1
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
    poseSubsystem.autoscoreCommandPose = reefPose
      /*** pathfinding ***/
    val returnCommand = AutoBuilder.pathfindToPose(
      reefPose,
      constraints,
      0.0,
    )

//    val plist = mutableListOf(PathPoint(
//      reefPose.translation, RotationTarget(0.0, reefPose.rotation), constraints
//    ))
//    println(reefPose.rotation)

//    val returnCommand = AutoBuilder.pathfindThenFollowPath(
//      PathPlannerPath.fromPathPoints
//      (
//        plist,
//        constraints,
//        GoalEndState(0.0, reefPose.rotation)
//        ),
//      constraints)

    return FollowPathCommandCooked(poseSubsystem, returnCommand)

  }

  /**
   * moves robot to processor location using
   * swerve drive.
   */
  private fun moveToProcessorCommand(): Command {
    println("processor command called")
    var processorPose = AutoScoreCommandConstants.processorPoseBlue

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      processorPose = AutoScoreCommandConstants.processorPoseRed
    }

    poseSubsystem.autoscoreCommandPose = processorPose

    val returnCommand = AutoBuilder.pathfindToPose(
      processorPose,
      constraints,
      0.0
    )

    val plist = mutableListOf(PathPoint(
      processorPose.translation, RotationTarget(0.0, processorPose.rotation), constraints
    ))
//    val returnCommand = AutoBuilder.pathfindThenFollowPath(
//      PathPlannerPath.fromPathPoints
//        (
//        plist,
//        constraints,
//        GoalEndState(0.0, processorPose.rotation)
//      ),
//      constraints)

    return FollowPathCommandCooked(poseSubsystem, returnCommand)
  }

  /**
   * moves robot to coral intake, either top
   * or bottom depending on the parameter passed
   * in, using swerve drive
   * @param isAtTopSource a boolean representing if we're intaking from the top or the bottom source. True if top, false if bottom.
   */
  private fun moveToCoralIntakeCommand(isAtTopSource: Boolean): Command {
    println("coral intake called")
    val coralIntakePose: Pose2d
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      if (isAtTopSource) {
        coralIntakePose = AutoScoreCommandConstants.coralIntakePoseRedTop
      } else {
        coralIntakePose = AutoScoreCommandConstants.coralIntakePoseRedBottom
      }
    } else {
      if (isAtTopSource) {
        coralIntakePose = AutoScoreCommandConstants.coralIntakePoseBlueTop
      } else {
        coralIntakePose = AutoScoreCommandConstants.coralIntakePoseBlueBottom
      }
    }
    poseSubsystem.autoscoreCommandPose = coralIntakePose

    val returnCommand = AutoBuilder.pathfindToPose(
      coralIntakePose,
      constraints,
      0.0
    )

    val plist = mutableListOf(PathPoint(
      coralIntakePose.translation, RotationTarget(0.0, coralIntakePose.rotation), constraints
    ))
//    val returnCommand = AutoBuilder.pathfindThenFollowPath(
//      PathPlannerPath.fromPathPoints
//        (
//        plist,
//        constraints,
//        GoalEndState(0.0, coralIntakePose.rotation)
//      ),
//      constraints)

    return FollowPathCommandCooked(poseSubsystem, returnCommand)
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
  private fun moveToNetCommand(onRedAllianceSide: Boolean): Command {
    var netPose = Pose2d(Translation2d(AutoScoreCommandConstants.centerOfField - AutoScoreCommandConstants.netTranslationDistance, poseSubsystem.pose.y), AutoScoreCommandConstants.netRotation2dBlue)
    if (onRedAllianceSide) {
      netPose = Pose2d(Translation2d(AutoScoreCommandConstants.centerOfField + AutoScoreCommandConstants.netTranslationDistance, poseSubsystem.pose.y), AutoScoreCommandConstants.netRotation2dRed)
    }

    poseSubsystem.autoscoreCommandPose = netPose
    println(poseSubsystem.autoscoreCommandPose.translation)
    val returnCommand = AutoBuilder.pathfindToPose(
      netPose,
      constraints,
      0.0
    )
//PathPoint(
//      netPose.translation, RotationTarget(0.0, netPose.rotation), constraints
//    )
//    val plist = mutableListOf<PathPoint>()
//    val returnCommand = AutoBuilder.pathfindThenFollowPath(
//      PathPlannerPath.fromPathPoints
//        (
//        plist,
//        constraints,
//        GoalEndState(0.0, netPose.rotation)
//      ),
//      constraints)

    return FollowPathCommandCooked(poseSubsystem, returnCommand)
  }

  /**
   * returns a command scores the coral on the reef
   * level that is passed in.
   * @param reefLevel a reefLevel enum that determines which level to score the coral on
   * does nothing right now
   */
  private fun premoveReefScore(reefLevel: AutoScoreCommandConstants.ReefLevel): Command {
    return when (reefLevel) {
      AutoScoreCommandConstants.ReefLevel.L1 -> robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE)
      AutoScoreCommandConstants.ReefLevel.L2 -> robot.superstructureManager.requestGoal(SuperstructureGoal.L2_PREMOVE)
      AutoScoreCommandConstants.ReefLevel.L3 -> robot.superstructureManager.requestGoal(SuperstructureGoal.L3_PREMOVE)
      AutoScoreCommandConstants.ReefLevel.L4 -> robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)
    }
  }

  private fun reefScore(reefLevel: AutoScoreCommandConstants.ReefLevel): Command {
    return when (reefLevel) {
      AutoScoreCommandConstants.ReefLevel.L1 -> robot.superstructureManager.requestGoal(SuperstructureGoal.L1)
      AutoScoreCommandConstants.ReefLevel.L2 -> robot.superstructureManager.requestGoal(SuperstructureGoal.L2)
      AutoScoreCommandConstants.ReefLevel.L3 -> robot.superstructureManager.requestGoal(SuperstructureGoal.L3)
      AutoScoreCommandConstants.ReefLevel.L4 -> robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
    }
  }

  /**
   * returns a command that scores into processor
   * does nothing right now
   * */
  private fun scoreProcessorCommand(): Command {
    //no processor currently
    return InstantCommand()
  }

  /**
   * returns a command that scores net into net
   * does nothing right now
   * */
  private fun scoreNetCommand(): Command {
    //no net currently
    return InstantCommand()
  }

  /**
   * intakes coral from coral station
   * */
  private fun intakeCoralCommand(): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
  }

  fun net(onRedAllianceSide: Boolean): Command {
    val netCommand = moveToNetCommand(onRedAllianceSide).andThen(scoreNetCommand())
    return netCommand
  }

  fun coral(atTopSource: Boolean): Command {
    val coralCommand = moveToCoralIntakeCommand(atTopSource).andThen(intakeCoralCommand())
    return coralCommand
  }

  fun reef(reefLocation: AutoScoreCommandConstants.ReefLocation, reefLevel: AutoScoreCommandConstants.ReefLevel): Command {
    val reefCommand = premoveReefScore(reefLevel).alongWith(moveToReefCommand(reefLocation)).andThen(reefScore(reefLevel))
    return reefCommand
  }

  fun processor(): Command {
    val processorCommand = PrintCommand("processor pose translation: ${AutoScoreCommandConstants.processorPoseRed}").andThen(moveToProcessorCommand()).andThen(scoreProcessorCommand())
    return processorCommand
  }
}
