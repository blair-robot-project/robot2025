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
import frc.team449.auto.choreo.MagnetizePIDPoseAlign
import frc.team449.subsystems.RobotConstants
import kotlin.math.floor

class Pathfinder(val robot: Robot) : SubsystemBase() {
  var ADStar = LocalADStar()
  var pathPub: StructArrayPublisher<Pose2d>? = null
  var velsPub: DoubleArrayPublisher? = null
  var pathSub: StructArraySubscriber<Pose2d>? = null
  var velsSub: DoubleArraySubscriber? = null
  var path: PathPlannerPath? = null
  var points: DoubleArray = doubleArrayOf(0.0, 0.0)
  private val timer = Timer()
  var expectedTime: Double? = 0.0
  var pathStart = 0.0
  var poseAfter: Pose2d = Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))
  var indexInPath = 0
  var speedNow: ChassisSpeeds = ChassisSpeeds(0.0, 0.0, 0.0)
  var otherspeednow: ChassisSpeeds = ChassisSpeeds(0.0, 0.0, 0.0)
  var pathRunning = false
  var pathValid = false
  var prevPose: Pose2d? = robot.poseSubsystem.pose
  var nextPose: Pose2d? = Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))
  var alongPath = 0.0
  var goalPos: Pose2d = Pose2d(Translation2d(0.0,0.0), Rotation2d(0.0))

  init {
    timer.restart()
    pathPub = NetworkTableInstance.getDefault().getStructArrayTopic("/activePath", Pose2d.struct).publish(*arrayOf<PubSubOption>())
    velsPub = NetworkTableInstance.getDefault().getDoubleArrayTopic("/pathvels").publish(*arrayOf<PubSubOption>())

    pathSub =
      NetworkTableInstance.getDefault().getStructArrayTopic("/activePath", Pose2d.struct)
        .subscribe(arrayOf(Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0)), Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))))
    velsSub = NetworkTableInstance.getDefault().getDoubleArrayTopic("/pathvels").subscribe(doubleArrayOf(0.0, 0.0))
  }

  override fun periodic() { // periodic is 2 loops ahead of path init
    // if (indexinpath+2 <= pathSub?.get()!!.size){
    pathRunning = (alongPath < 1.0)
    pathValid = ((pathSub?.get()?.get(0) ?: Pose2d()) != Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0)))
    println("pathrunning $pathRunning")
    println("pathvalid $pathValid") //
    if (pathRunning && pathValid) {
      try {
        expectedTime = path?.generateTrajectory(robot.drive.currentSpeeds, robot.poseSubsystem.pose.rotation, RobotConfig.fromGUISettings())?.totalTimeSeconds!!
        println("time now in periodic: ${timer.get()}")
        println("pathstart: $pathStart")
        println("expectedtime: $expectedTime")
        alongPath = (timer.get() - pathStart) / expectedTime!!
        println("alongpath: $alongPath")
        var numsections = pathSub?.get()!!.size - 1
        println("num sections: $numsections")
        println("index thing: ${floor(alongPath * numsections) + 1}")
        indexInPath = (floor(alongPath * numsections)).toInt() + 1
        if (alongPath < 1.0) {
          prevPose = pathSub?.get()?.get(indexInPath - 1)
          nextPose = pathSub?.get()?.get(indexInPath) //
          if (prevPose != null) {
            if (nextPose != null) {
              speedNow = MagnetizePIDPoseAlign(robot.drive, robot.poseSubsystem, goalPos, robot.driveController).calculate(robot.poseSubsystem.pose, nextPose!!)
              println("speed now: ${speedNow * 1.0}")
              robot.drive.set(speedNow *100000000000000000000000000000000000000000000000.0)
            }
          }
          println()
        }
//        if (indexinpath == pathSub?.get()!!.size) {
//          println("done")
//          alongpath = 0.0
//        }
      } catch (e: Exception) {
        println("path done probably")
        alongPath = 0.0
      }
    }

    if (ADStar.isNewPathAvailable) {
      path = ADStar.getCurrentPath(
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
        for (point in path!!.allPathPoints) {
          points += (point.maxV)
        }
        pathPub?.set(path!!.pathPoses.toTypedArray<Pose2d>())
        velsPub?.set(points)
      }
      pathRunning = true
      pathValid = true
    }
  }

  fun path(goalPosition: Pose2d): Command {
    // println("time now in path command: ${timer.get()}")
    // println("index thing: $indexinpath")
    return runOnce({
      goalPos = goalPosition
      alongPath = 0.0
      pathStart = timer.get() //
      // println(robot.poseSubsystem.pose.translation)
      ADStar.setStartPosition(robot.poseSubsystem.pose.translation)
      ADStar.setGoalPosition(goalPosition.translation)
    }).andThen(PrintCommand("                                                                                                              goal updated")) //
    // .andThen(PrintCommand("pathstart: $pathstart")).andThen(PrintCommand(" "))
  }
}
