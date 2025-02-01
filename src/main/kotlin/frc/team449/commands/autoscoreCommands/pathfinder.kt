package frc.team449.commands.autoscoreCommands

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.path.PathPoint
import com.pathplanner.lib.pathfinding.LocalADStar
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
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
    adstar.setStartPosition(rightpoint(Translation2d(robot.poseSubsystem.getPosea().translation.x,robot.poseSubsystem.getPosea().translation.y)))
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

  fun testpath(): Command {
    robot.poseSubsystem.pose = Pose2d(Translation2d(1.0, 2.0), Rotation2d(0.0))
    var endstate = GoalEndState(0.0, Rotation2d(0.0))
    adstar.setStartPosition(rightpoint(Translation2d(1.0, 2.0)))
    adstar.setGoalPosition(Translation2d(8.0, 4.0))
    var pathh: PathPlannerPath? = null
    while (pathh == null) {
      pathh = adstar.getCurrentPath(constraints, endstate)
    }
    println("path, ${adstar.getCurrentPath(constraints, endstate).numPoints()} waypoints")
    return AutoBuilder.followPath(pathh)
  }

  fun rightpoint(point: Translation2d): Translation2d {
    //return Translation2d(17.548225-point.x, 8.0518-point.y)
    //return Translation2d(point.x+17.548225, point.y+8.0518)
    return Translation2d(2.0,2.0)
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