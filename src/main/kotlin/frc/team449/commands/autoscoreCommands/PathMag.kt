package frc.team449.commands.autoscoreCommands

import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.pathfinding.LocalADStar
import com.pathplanner.lib.trajectory.PathPlannerTrajectory
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.networktables.*
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Robot
import frc.team449.subsystems.RobotConstants

class PathMag(val robot: Robot): SubsystemBase() {
  var adStar = LocalADStar()
  private var pathPub: StructArrayPublisher<Pose2d>? = null
  private var pathSub: StructArraySubscriber<Pose2d>? = null
  private var path: PathPlannerPath? = null
  private val timer = Timer()
  private var expectedTime: Double? = 0.0
  private var startTime = 0.0
  private var currentSpeed: ChassisSpeeds = ChassisSpeeds(0.0, 0.0, 0.0)
  private var pathRunning = false
  private var pathValid = false
  private var goalPos: Pose2d = Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))
  private var trajectory: PathPlannerTrajectory? = null

  init {
    timer.restart()
    pathPub = NetworkTableInstance.getDefault().getStructArrayTopic("/ADStarPath", Pose2d.struct).publish(*arrayOf<PubSubOption>())
    pathSub =
      NetworkTableInstance.getDefault().getStructArrayTopic("/ADStarPath", Pose2d.struct)
        .subscribe(arrayOf(Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0)), Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))))
  }

  override fun periodic() {
    if (adStar.isNewPathAvailable) {
      println("new path")
      path = adStar.getCurrentPath(
        PathConstraints(
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ACCEL,
          RobotConstants.MAX_ROT_SPEED,
          RobotConstants.ROT_RATE_LIMIT
        ),
        GoalEndState(0.0, robot.poseSubsystem.pose.rotation)
      )
      if (path != null) {
        println("new path, not null")
        println("new trajectory")
        trajectory = path!!.generateTrajectory(
          robot.drive.currentSpeeds,
          robot.poseSubsystem.pose.rotation,
          RobotConfig.fromGUISettings()
        )
        expectedTime = trajectory!!.totalTimeSeconds
        pathPub?.set(path!!.pathPoses.toTypedArray<Pose2d>())
        pathRunning = (robot.poseSubsystem.pose != goalPos)
        pathValid = ((pathSub?.get()?.get(0) ?: Pose2d()) != Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0,0.0)))
        startTime = timer.get() //
      }
    }
    if (pathRunning && pathValid) {
      if (trajectory == null) {
        println("trajectory null")
      }
      if (trajectory != null) {
        if (path == null) {
          println("path null")
        }
        expectedTime = trajectory?.totalTimeSeconds
        if (timer.get() - startTime < expectedTime!!) {
          println("time now in periodic: ${timer.get()}")
          println("path start: $startTime")
          println("expected time: $expectedTime")
          currentSpeed = trajectory?.sample(timer.get() - startTime)?.fieldSpeeds ?: robot.drive.currentSpeeds
          println("time since start: ${timer.get() - startTime}")
          println("speed now: $currentSpeed")
          val setCommand = RunCommand({
            robot.poseSubsystem.pathfindingMagnetize(currentSpeed)
          }).withTimeout(0.02)
          setCommand.schedule()
          pathRunning = (robot.poseSubsystem.pose != goalPos)
          pathValid = ((pathSub?.get()?.get(0) ?: Pose2d()) != Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0)))
          println("path running: $pathRunning")
          println("path valid: $pathValid")
          println()
        } else {
          expectedTime = 0.0
        }
        robot.drive.set(ChassisSpeeds(0.0, 0.0, 0.000000001))
        println("rotation: ${robot.poseSubsystem.pose.rotation}")
        adStar.setStartPosition(robot.poseSubsystem.pose.translation)
      }
    }
  }
  fun getPath(goalPosition: Pose2d): Command {
    return runOnce {
      goalPos = goalPosition
      startTime = timer.get() //
      adStar.setStartPosition(robot.poseSubsystem.pose.translation)
      adStar.setGoalPosition(goalPosition.translation)
    }.andThen(PrintCommand("\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t goal updated")) //

  }
}
