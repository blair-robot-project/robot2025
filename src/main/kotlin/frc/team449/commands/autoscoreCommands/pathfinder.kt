package frc.team449.commands.autoscoreCommands

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.FollowPathCommand
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
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Robot
import frc.team449.subsystems.RobotConstants
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.auto.choreo.MagnetizePIDPoseAlign
import kotlin.math.floor

class pathfinder(val robot: Robot) : SubsystemBase() {
  var adstar = LocalADStar()
  var pathPub: StructArrayPublisher<Pose2d>? = null
  var velsPub: DoubleArrayPublisher? = null
  var pathSub: StructArraySubscriber<Pose2d>? = null
  var velsSub: DoubleArraySubscriber? = null
  var path: PathPlannerPath? = null
  var points: DoubleArray = doubleArrayOf(0.0, 0.0)
  private val timer = Timer()
  var expectedtime = 0.0
  var pathstart = 0.0
  var poseafter: Pose2d = Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))
  var indexinpath = 0
  var speednow: ChassisSpeeds = ChassisSpeeds(0.0,0.0,0.0)
  var otherspeednow: ChassisSpeeds = ChassisSpeeds(0.0,0.0,0.0)
  var pathrunning = false
  var pathvalid = false
  var prevpose: Pose2d? = robot.poseSubsystem.pose
  var nextpose: Pose2d? = Pose2d(Translation2d(0.0,0.0), Rotation2d(0.0))
  var alongpath = 0.0

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
    //if (indexinpath+2 <= pathSub?.get()!!.size){
    pathrunning = (alongpath < 1.0)
    pathvalid =  ((pathSub?.get()?.get(0) ?: Pose2d()) != Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0)))
//    println("pathrunning $pathrunning")
//    println("pathvalid $pathvalid")
    if (pathrunning && pathvalid) {
      expectedtime = path?.generateTrajectory(robot.drive.currentSpeeds, robot.poseSubsystem.pose.rotation, RobotConfig.fromGUISettings())?.totalTimeSeconds!!
      println("time now in periodic: ${timer.get()}")
      println("pathstart: $pathstart")
      println("expectedtime: $expectedtime")
      alongpath = (timer.get() - pathstart) / expectedtime
      println("alongpath: $alongpath")
      var numsections = pathSub?.get()!!.size - 1
      println("num sections: $numsections")
      println("index thing: ${floor(alongpath * numsections) + 1}")
      indexinpath = (floor(alongpath * numsections)).toInt()+1
      if (alongpath < 1.0) {
        prevpose = pathSub?.get()?.get(indexinpath-1)
        nextpose = pathSub?.get()?.get(indexinpath)//
        if (prevpose != null) {
          if (nextpose != null) {
            speednow = MagnetizePIDPoseAlign(robot.drive, robot.poseSubsystem, robot.poseSubsystem.pose, robot.driveController).calculate(robot.poseSubsystem.pose, nextpose!!)
            println("speed now: ${speednow*1.0}")
            robot.drive.set(speednow*1.0)
          }
        }
        println()
      }
      if (indexinpath == pathSub?.get()!!.size) {
        println("done")
        alongpath = 0.0
      }
    }

    if (adstar.isNewPathAvailable) {
      path = adstar.getCurrentPath(
        PathConstraints(
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ACCEL,
          RobotConstants.MAX_ROT_SPEED,
          RobotConstants.ROT_RATE_LIMIT
        ), GoalEndState(0.0, robot.poseSubsystem.pose.rotation)
      )
      if (path != null) {
        alongpath=0.0
        for (point in path!!.allPathPoints) {
          points += (point.maxV)
        }
        pathPub?.set(path!!.pathPoses.toTypedArray<Pose2d>())
        velsPub?.set(points)
      }
      pathrunning = true
      pathvalid = true
    }
  }

  fun path(goalPosition: Pose2d): Command {
    //println("time now in path command: ${timer.get()}")
    //println("index thing: $indexinpath")
    return runOnce({
      alongpath = 0.0
      pathstart = timer.get()//
      //println(robot.poseSubsystem.pose.translation)
      adstar.setStartPosition(robot.poseSubsystem.pose.translation)
      adstar.setGoalPosition(goalPosition.translation)
    }).andThen(PrintCommand("                                                                                                              goal updated"))//
    //.andThen(PrintCommand("pathstart: $pathstart")).andThen(PrintCommand(" "))
  }
}