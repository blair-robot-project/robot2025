package frc.team449.commands.autoscoreCommands

import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.pathfinding.LocalADStar
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.networktables.*
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Robot
import frc.team449.commands.driveAlign.PIDPoseAlign
import frc.team449.subsystems.RobotConstants
import kotlin.math.floor

class PathMag(val robot: Robot): SubsystemBase() {
  var adStar = LocalADStar()
  private var pathPub: StructArrayPublisher<Pose2d>? = null
  private var pathSub: StructArraySubscriber<Pose2d>? = null
  private var path: PathPlannerPath? = null
  private val timer = Timer()
  private var expectedTime: Double? = 0.0
  private var startTime = 0.0
  private var pathIndex = 0
  private var currentSpeed: ChassisSpeeds = ChassisSpeeds(0.0, 0.0, 0.0)
  private var pathRunning = false
  private var pathValid = false
  private var prevPose: Pose2d? = robot.poseSubsystem.pose
  private var nextPose: Pose2d? = Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))
  private var alongPath = 0.0
  private var goalPos: Pose2d = Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))

  init {
    timer.restart()
    pathPub = NetworkTableInstance.getDefault().getStructArrayTopic("/ADStarPath", Pose2d.struct).publish(*arrayOf<PubSubOption>())
    pathSub =
      NetworkTableInstance.getDefault().getStructArrayTopic("/ADStarPath", Pose2d.struct)
        .subscribe(arrayOf(Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0)), Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))))
  }

  override fun periodic() {
    if (adStar.isNewPathAvailable) {
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
        alongPath = 0.0
        pathIndex = 0
        expectedTime = 0.0
        pathPub?.set(path!!.pathPoses.toTypedArray<Pose2d>())
        expectedTime = path?.generateTrajectory(robot.drive.currentSpeeds, robot.poseSubsystem.pose.rotation, RobotConfig.fromGUISettings())?.totalTimeSeconds!!
        pathRunning = (robot.poseSubsystem.pose != goalPos)
        pathValid = ((pathSub?.get()?.get(0) ?: Pose2d()) != Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0)))
      }
    }
    if (pathRunning && pathValid) {
      if (path == null) {
        println("path null")
      }
      expectedTime = path?.generateTrajectory(robot.drive.currentSpeeds, robot.poseSubsystem.pose.rotation, RobotConfig.fromGUISettings())?.totalTimeSeconds!!
      println("time now in periodic: ${timer.get()}")
      println("path start: $startTime")
      println("expected time: $expectedTime")
      alongPath = (timer.get() - startTime) / expectedTime!!
      println("along path: $alongPath")
      val numSections = pathSub?.get()!!.size - 1
      println("num sections: $numSections")
      println("index thing: ${floor(alongPath * numSections) + 1}")
      pathIndex = (floor(alongPath * numSections)).toInt() + 1
      if (pathIndex<numSections-1) {
        prevPose = pathSub?.get()?.get(pathIndex - 1)
        nextPose = pathSub?.get()?.get(pathIndex) //
        if (prevPose != null) {
          if (nextPose != null) {
            currentSpeed = PIDPoseAlign(robot.drive, robot.poseSubsystem, nextPose!!, 100.0, 100.0).calculate(robot.poseSubsystem.pose, nextPose!!)
            println("speed now: $currentSpeed")
            robot.drive.set(currentSpeed)
          }
        }
        pathRunning = (robot.poseSubsystem.pose != goalPos)
        pathValid = ((pathSub?.get()?.get(0) ?: Pose2d()) != Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0)))
        println("path running: $pathRunning")
        println("path valid: $pathValid")
        println()
      }
      else {
        println("path done probably")
        alongPath = 0.0
        pathIndex = 0
        expectedTime = 0.0
        prevPose = Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))
        nextPose = Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))
      }
    } else {
      println("path done probably")
      alongPath = 0.0
      pathIndex = 0
      expectedTime = 0.0
      prevPose = Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))
      nextPose = Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))
    }

  }
    fun getPath(goalPosition: Pose2d): Command {
      return runOnce {
        alongPath = 0.0
        pathIndex = 0
        prevPose = Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))
        nextPose = Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))
        goalPos = goalPosition
        startTime = timer.get() //
        // println(robot.poseSubsystem.pose.translation)
        adStar.setStartPosition(robot.poseSubsystem.pose.translation)
        adStar.setGoalPosition(goalPosition.translation)
//      if (pathRunning&&pathValid) {
//        expectedTime = path?.generateTrajectory(robot.drive.currentSpeeds, robot.poseSubsystem.pose.rotation, RobotConfig.fromGUISettings())?.totalTimeSeconds!!
//      }
      }.andThen(PrintCommand("\t\t\t\t\t goal updated")) //

  }
}
