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
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Robot
import frc.team449.commands.driveAlign.PIDPoseAlign
import frc.team449.subsystems.RobotConstants
import kotlin.math.floor

class AutoScorePathfinder(val robot: Robot) : SubsystemBase() {
  var ADStar = LocalADStar()
  //lateinit is necessary here
  private var pathPub: StructArrayPublisher<Pose2d>
  private var velXPub: DoublePublisher
  private var velYPub: DoublePublisher
  private var velRotationPub: DoublePublisher
  private var pathSub: StructArraySubscriber<Pose2d>
  private var velXSub: DoubleSubscriber
  private var velYSub: DoubleSubscriber
  private var velRotationSub: DoubleSubscriber
  private lateinit var path: PathPlannerPath
  private var velocityX = 0.0
  private var velocityY = 0.0
  private var rotation = 0.0
  private val timer = Timer()
  private var ET = 0.0 //expected time
  private var startingTime = 0.0
  private val zeroPose = Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))
  private var poseEnd = zeroPose
  private var pathIndex = 0
  private var lastTimeCalled = 0.0
  private var pathActive = false
  private var pathCompletion = 0.0
  private var poseAlign = PIDPoseAlign(robot.drive, robot.poseSubsystem, zeroPose, AutoScoreCommandConstants.MAX_LINEAR_SPEED, AutoScoreCommandConstants.MAX_ROT_SPEED)
  private var currentCommand : Command = InstantCommand()

  init {
    timer.restart()
    pathPub = NetworkTableInstance.getDefault().getStructArrayTopic("/activePath", Pose2d.struct).publish(*arrayOf<PubSubOption>())
    velXPub = NetworkTableInstance.getDefault().getDoubleTopic("/pathVelocityX").publish()
    velYPub = NetworkTableInstance.getDefault().getDoubleTopic("/pathVelocityY").publish()
    velRotationPub = NetworkTableInstance.getDefault().getDoubleTopic("/pathRotation").publish()

    pathSub =
      NetworkTableInstance.getDefault().getStructArrayTopic("/activePath", Pose2d.struct)
        .subscribe(arrayOf(zeroPose, zeroPose))

    velXSub = NetworkTableInstance.getDefault().getDoubleTopic("/pathVelocityX").subscribe(0.0)
    velYSub = NetworkTableInstance.getDefault().getDoubleTopic("/pathVelocityY").subscribe(0.0)
    velRotationSub = NetworkTableInstance.getDefault().getDoubleTopic("/pathRotation").subscribe(0.0)
    PathPlannerPath.clearCache()
  }

  private fun resetVelocity() {
    velocityX = 0.0
    velocityY = 0.0
    rotation = 0.0
  }

  private fun resetVars() {
    pathCompletion = 0.0
    resetVelocity()
    poseEnd = zeroPose
  }

  override fun periodic() {
    val currentTime = timer.get()
    val dt = currentTime - lastTimeCalled
    lastTimeCalled = currentTime
    if (dt > 0.2) /*command called a new time*/ {
      startingTime = timer.get()
    }
    val sections = pathSub.get().size - 1
    if(pathCompletion >= 1) {
      pathActive = false
      currentCommand.cancel()
    }
    if(pathActive) {
      if ((pathSub.get()[0]) != zeroPose) {
        ET = (pathSub.get().size - 1) * 0.02 //num sections*time/section
        println("ct: $currentTime")
        println("st: $startingTime, et: $ET")
        // pathCompletion ranging from 0 to 1
        pathCompletion = (timer.get() - startingTime) / ET
        pathIndex = floor(pathCompletion * sections).toInt()
        val prevPose = pathSub.get()[pathIndex - 1]
        val nextPose = pathSub.get()[pathIndex] //
        if (prevPose != null) {
          if (nextPose != null) {
            val currentSpeed = poseAlign.calculate(prevPose, nextPose)
            robot.poseSubsystem.pathfindingMagnetize(currentSpeed)
            velocityX = currentSpeed.vxMetersPerSecond
            velocityY = currentSpeed.vyMetersPerSecond
            rotation = currentSpeed.omegaRadiansPerSecond
            velXPub.set(velocityX)
            velYPub.set(velocityY)
            velRotationPub.set(rotation)
          }
        }
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
        if(path != null) {
          pathPub.set(path.pathPoses.toTypedArray<Pose2d>())
          resetVelocity()
          pathCompletion = 0.0
        }
      }
    }

  }

  fun path(goalPosition: Pose2d): Command {
    //println("time now in path command: ${timer.get()}")
    //println("index thing: $indexinpath")
    currentCommand = runOnce {
      resetVars()
      ADStar.setStartPosition(robot.poseSubsystem.pose.translation)
      ADStar.setGoalPosition(goalPosition.translation)
      pathActive = true
    }.andThen(PrintCommand("goal updated"))
    return currentCommand
  }
}