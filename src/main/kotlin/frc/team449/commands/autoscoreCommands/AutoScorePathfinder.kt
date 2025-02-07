package frc.team449.commands.autoscoreCommands

import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.pathfinding.LocalADStar
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.networktables.*
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Robot
import frc.team449.subsystems.RobotConstants
import kotlin.math.floor

class AutoScorePathfinder(val robot: Robot) : SubsystemBase() {
  var ADStar = LocalADStar()
  //lateinit is necessary here
  private lateinit var pathPub: StructArrayPublisher<Pose2d>
  private lateinit var velsPub: DoubleArrayPublisher
  private lateinit var pathSub: StructArraySubscriber<Pose2d>
  private lateinit var velsSub: DoubleArraySubscriber
  private lateinit var path: PathPlannerPath
  private var velocities: DoubleArray = doubleArrayOf()
  private val timer = Timer()
  private var ET = 0.0 //expected time
  private var startingTime = 0.0
  private val zeroPose = Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))
  private var poseEnd = zeroPose
  private var pathIndex = 0
  private var lastTimeCalled = 0.0

  init {
    timer.restart()
    pathPub = NetworkTableInstance.getDefault().getStructArrayTopic("/activePath", Pose2d.struct).publish(*arrayOf<PubSubOption>())
    velsPub = NetworkTableInstance.getDefault().getDoubleArrayTopic("/pathVelocities").publish(*arrayOf<PubSubOption>())

    pathSub =
      NetworkTableInstance.getDefault().getStructArrayTopic("/activePath", Pose2d.struct)
        .subscribe(arrayOf(zeroPose, zeroPose))
    velsSub = NetworkTableInstance.getDefault().getDoubleArrayTopic("/pathVelocities").subscribe(doubleArrayOf())
    PathPlannerPath.clearCache()
  }

  private fun resetVelocities() {
    velocities = doubleArrayOf()
  }

  override fun periodic() {
    val currentTime = timer.get()
    val dt = currentTime - lastTimeCalled
    lastTimeCalled = currentTime
    if (dt > 0.2) /*command called a new time*/ {
      startingTime = timer.get()
    }
    val sections = pathSub.get().size - 1
    if ((pathSub.get()[0]) != zeroPose) {
      ET = (pathSub.get().size - 1) * 0.02 //num sections*time/section
      println("ct: $currentTime")
      println("st: $startingTime, et: $ET")
      // pathCompletion ranging from 0 to 1
      val pathCompletion = (timer.get() - startingTime) / ET
      pathIndex = floor(pathCompletion * sections).toInt()
      println("index thing: $pathIndex")
    }
    if (ADStar.isNewPathAvailable) {
      path = ADStar.getCurrentPath(
        PathConstraints(
          RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ACCEL,
          RobotConstants.MAX_ROT_SPEED,
          RobotConstants.ROT_RATE_LIMIT
        ), GoalEndState(0.0, robot.poseSubsystem.pose.rotation)
      )
      resetVelocities()
      for (point in path.allPathPoints) {
        velocities += (point.maxV)
      }
      pathPub.set(path.pathPoses.toTypedArray<Pose2d>())
      velsPub.set(velocities)

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
    if (velsSub.get() != null) {
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
      ADStar.setStartPosition(robot.poseSubsystem.pose.translation)
      ADStar.setGoalPosition(goalPosition.translation)
    }).andThen(PrintCommand("goal updated"))
    //.andThen(PrintCommand("pathstart: $pathstart")).andThen(PrintCommand(" "))
  } }