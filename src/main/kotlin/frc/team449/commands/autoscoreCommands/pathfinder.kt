package frc.team449.commands.autoscoreCommands

import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.pathfinding.LocalADStar
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.networktables.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Robot
import frc.team449.subsystems.RobotConstants
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.InstantCommand
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

  init {
    timer.restart()
    pathPub = NetworkTableInstance.getDefault().getStructArrayTopic("/activePath", Pose2d.struct).publish(*arrayOf<PubSubOption>())
    velsPub = NetworkTableInstance.getDefault().getDoubleArrayTopic("/pathvels").publish(*arrayOf<PubSubOption>())

    pathSub =
      NetworkTableInstance.getDefault().getStructArrayTopic("/activePath", Pose2d.struct)
        .subscribe(arrayOf(Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0)), Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))))
    velsSub = NetworkTableInstance.getDefault().getDoubleArrayTopic("/pathvels").subscribe(doubleArrayOf(0.0, 0.0))
  }

  override fun periodic() {
    //println("first pose: ${pathSub?.get()?.get(0)}")
    //println()
    if ((pathSub?.get()?.get(0) ?: Pose2d()) != Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))) {
      expectedtime = (pathSub?.get()!!.size - 1) * 0.02 //num sections*time/section
      println("time now in periodic: ${timer.get()}")
      println("pathstart: $pathstart")
      println("expectedtime: $expectedtime")
      var alongpath = (timer.get() - pathstart) / expectedtime
      println("alongpath: $alongpath")
      println("num sections: ${pathSub?.get()!!.size - 1}")
      indexinpath = floor(alongpath / (1 / (pathSub?.get()!!.size - 1))).toInt() + 1
      println("index thing: $indexinpath")
      println()
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
//        expectedtime = (pathSub?.get()!!.size - 1) * 0.02 //num sections*time/section
        for (point in path!!.allPathPoints) {
          points += (point.maxV)
        }
        pathPub?.set(path!!.pathPoses.toTypedArray<Pose2d>())
        velsPub?.set(points)

      }
    }
//    if (pathSub?.get()!=null) {
//      println("num poses: ${pathSub?.get()!!.size}")
//      println()
//      for (pose in pathSub?.get()!!) {
//        print("pose: $pose, ")
//      }
//      println()
//      println()
//      println()
//      println()
//    }
    if (velsSub?.get() != null) {
//      for (vel in velsSub?.get()!!) {
//        print("vel: $vel")
//      }
      //println("random timestamp thing hopefully at the beginning of the path: ${velsSub?.atomic?.timestamp}")
      //println()
      //println()
      //println()
      //println()
    }
  }

  fun path(goalPosition: Pose2d): Command {
    //println("time now in path command: ${timer.get()}")
    //println("index thing: $indexinpath")
    return runOnce({
      pathstart = timer.get()+0.04
      //println(robot.poseSubsystem.pose.translation)
      adstar.setStartPosition(robot.poseSubsystem.pose.translation)
      adstar.setGoalPosition(goalPosition.translation)
    }).andThen(PrintCommand("goal updated"))
    //.andThen(PrintCommand("pathstart: $pathstart")).andThen(PrintCommand(" "))
  } }