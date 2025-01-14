package frc.team449.commands.autoscoreCommands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.auto.choreo.MagnetizePIDPoseAlign
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.vision.PoseSubsystem
import kotlin.math.*

class AutoScoreCommands(
  private val drive: SwerveDrive,
  private val poseSubsystem: PoseSubsystem,
  private val controller: XboxController
) {

  init {
  }
  private var prevX = 0.0
  private var prevY = 0.0

  private var prevTime = 0.0

  private var dx = 0.0
  private var dy = 0.0
  private var magAcc = 0.0
  private var dt = 0.0
  private var magAccClamped = 0.0

  private var rotScaled = 0.0

  var headingLock = false

  private var rotRamp = SlewRateLimiter(RobotConstants.ROT_RATE_LIMIT)

  private val timer = Timer()

  private val rotCtrl = PIDController(
    RobotConstants.SNAP_KP,
    RobotConstants.SNAP_KI,
    RobotConstants.SNAP_KD
  )

  /**
   * This command moves the robot to one of the twelve reef locations
   * (location 1 is the most vertical location on the right, going
   * clockwise)
   * @param reefLocation a reefLocationEnum that defines which spot to go to, defined with the numeric system above.
   */
  fun moveToReefCommand(
    reefLocation: AutoScoreCommandConstants.reefLocation
  ): Command {
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

    return MagnetizePIDPoseAlign(drive, poseSubsystem, reefPose, controller)
  }

  /**
   * moves robot to processor location using
   * swerve drive.
   */
  fun moveToProcessorCommand(): Command {
    // val pose2d = Pose2d(AutoScoreCommandConstants.processorTranslation2dBlue,AutoScoreCommandConstants.processorRotation2dBlue)
    // var returnCommand = PIDPoseAlign(drive, poseSubsystem, AutoScoreCommandConstants.processorPoseBlue)
    var returnCommand = MagnetizePIDPoseAlign(
      drive,
      poseSubsystem,
      AutoScoreCommandConstants.processorPoseBlue,
      controller
    )
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      returnCommand = MagnetizePIDPoseAlign(drive, poseSubsystem, AutoScoreCommandConstants.processorPoseRed, controller)
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
    var returnCommand = MagnetizePIDPoseAlign(drive, poseSubsystem, AutoScoreCommandConstants.coralIntakePoseBlueTop, controller)
    if (!isAtTopSource) {
      returnCommand = MagnetizePIDPoseAlign(drive, poseSubsystem, AutoScoreCommandConstants.coralIntakePoseBlueBottom, controller)
    }
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      returnCommand = MagnetizePIDPoseAlign(drive, poseSubsystem, AutoScoreCommandConstants.coralIntakePoseRedTop, controller)
      if (!isAtTopSource) {
        returnCommand = MagnetizePIDPoseAlign(drive, poseSubsystem, AutoScoreCommandConstants.coralIntakePoseRedBottom, controller)
      }
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
    var netPose = Pose2d(Translation2d(AutoScoreCommandConstants.centerOfField - AutoScoreCommandConstants.netTranslationDistance, poseSubsystem.pose.y), AutoScoreCommandConstants.netRotation2dBlue)
    if (onRedAllianceSide) {
      netPose = Pose2d(Translation2d(AutoScoreCommandConstants.centerOfField + AutoScoreCommandConstants.netTranslationDistance, poseSubsystem.pose.y), AutoScoreCommandConstants.netRotation2dBlue)
    }
    return MagnetizePIDPoseAlign(drive, poseSubsystem, netPose, controller)
  }

  /**
   * returns a command scores the coral on the reef
   * level that is passed in.
   * @param reefLevel a reefLevel enum that determines which level to score the coral on
   * does nothing right now
   */
  fun putCoralInReef(reefLevel: AutoScoreCommandConstants.reefLevel): Command {
    // we don't have score yet, but we're setting up stuff for future
    // we won't have to account for alliance here
    when (reefLevel) {
      AutoScoreCommandConstants.reefLevel.L1 -> println("scoring coral l1")
      AutoScoreCommandConstants.reefLevel.L2 -> println("scoring coral l2")
      AutoScoreCommandConstants.reefLevel.L3 -> println("scoring coral l3")
      AutoScoreCommandConstants.reefLevel.L4 -> println("scoring coral l4")
    }
    // returns a command that does nothing (for now)
    return InstantCommand()
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
   * intakes coral from coral intake
   * does nothing right now
   * */
  fun intakeCoralCommand(): Command {
    // we don't have coral intake yet, just setting up.
    // returns a command that does nothing (for now)
    return InstantCommand()
  }
}
