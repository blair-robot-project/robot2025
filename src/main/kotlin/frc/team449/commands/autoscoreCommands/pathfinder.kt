package frc.team449.commands.autoscoreCommands

import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.pathfinding.LocalADStar
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.PubSubOption
import edu.wpi.first.networktables.StructArrayPublisher
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Robot
import frc.team449.subsystems.RobotConstants

class pathfinder(val robot: Robot) : SubsystemBase(){
  var adstar = LocalADStar()
  var pathPub: StructArrayPublisher<Pose2d>? = null
  var path: PathPlannerPath? = null

  init {
    pathPub = NetworkTableInstance.getDefault().getStructArrayTopic("/activePath", Pose2d.struct).publish(*arrayOf<PubSubOption>())
  }

  override fun periodic() {
    if (adstar.isNewPathAvailable) {
      path = adstar.getCurrentPath(
        PathConstraints(RobotConstants.MAX_LINEAR_SPEED,
          RobotConstants.MAX_ACCEL,
          RobotConstants.MAX_ROT_SPEED,
          RobotConstants.ROT_RATE_LIMIT), GoalEndState(0.0, robot.poseSubsystem.pose.rotation))
      if (path != null) {
        pathPub?.set(path!!.pathPoses.toTypedArray<Pose2d>())
      }
    }
  }

  fun path (goalPosition: Pose2d): Command {
    return runOnce({
      println(robot.poseSubsystem.pose.translation)
      adstar.setStartPosition(robot.poseSubsystem.pose.translation)
      adstar.setGoalPosition(goalPosition.translation)
    }).andThen(PrintCommand("goal updated"))
  }
}
