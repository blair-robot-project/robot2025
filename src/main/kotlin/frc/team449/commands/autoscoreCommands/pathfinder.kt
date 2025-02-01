package frc.team449.commands.autoscoreCommands

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.path.PathPoint
import com.pathplanner.lib.pathfinding.LocalADStar
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.team449.Robot
import java.nio.file.Path

class pathfinder(val robot: Robot) {
  var adstar = LocalADStar()
  val constraints = PathConstraints(
    3.0,
    4.0,
    Units.degreesToRadians(540.0),
    Units.degreesToRadians(720.0)
  )

  fun path (goalPosition: Pose2d): Command {
    var endstate = GoalEndState(0.0, goalPosition.rotation)
    println("current pose: ${robot.poseSubsystem.getPosea()}")
    adstar.setStartPosition(robot.poseSubsystem.getPosea().translation)
    adstar.setGoalPosition(goalPosition.translation)
    var pathh: PathPlannerPath? = null
    while (pathh == null) {
      pathh = adstar.getCurrentPath(constraints, endstate)
    }
    adstar.setStartPosition(robot.poseSubsystem.getPosea().translation)
    adstar.setGoalPosition(goalPosition.translation)
    println("current pose: ${robot.poseSubsystem.getPosea()}")
    println("path, ${adstar.getCurrentPath(constraints, endstate).numPoints()} waypoints")
    return AutoBuilder.followPath(pathh)
  }

  fun pathpoints (goalPosition: Pose2d): MutableList<PathPoint>? {
    var endstate = GoalEndState(0.0, goalPosition.rotation)
      adstar.setStartPosition(robot.poseSubsystem.getPosea().translation)
      adstar.setGoalPosition(goalPosition.translation)
    var pathh: PathPlannerPath? = null
    while (pathh == null) {
      pathh = adstar.getCurrentPath(constraints, endstate)
    }
    println("path, ${adstar.getCurrentPath(constraints, endstate).numPoints()} waypoints")
    println("waypoints: ")
    for (waypoint in pathh.allPathPoints) {
      println("waypoint $waypoint")
    }
    return pathh.allPathPoints
  }

  /** all points in a path */
  fun points (path: PathPlannerPath): MutableList<PathPoint>? {
    println("num path points: ${path.numPoints()}")
    return path.allPathPoints
  }

  /** all poses in a path */
  fun poses (path: PathPlannerPath): MutableList<Pose2d>? {
    return path.pathPoses
  }
}