package frc.team449.commands.autoscoreCommands

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.pathfinding.LocalADStar
import com.pathplanner.lib.pathfinding.Pathfinding
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.*
import frc.team449.Robot

class pathfinder(val robot: Robot) : SubsystemBase(){
  var adstar = LocalADStar()
  val constraints = PathConstraints(
    3.0,
    4.0,
    Units.degreesToRadians(540.0),
    Units.degreesToRadians(720.0)
  )

  init {
    Pathfinding.setPathfinder(adstar)
    AutoBuilder.configure(
      robot.poseSubsystem::getPosea, // poseSupplier - a supplier for the robot's current pose
      robot.poseSubsystem::resetOdometry, // resetPose - a consumer for resetting the robot's pose
      robot.drive::getCurrentSpeedsa, // robotRelativeSpeedsSupplier - a supplier for the robot's current robot relative chassis speeds
      robot.drive::set, // output - Output function that accepts robot-relative ChassisSpeeds
      PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        PIDConstants(5.0, 0.0, 0.0), // Translation PID constants, placeholders
        PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants, placeholders
      ),
      RobotConfig.fromGUISettings(),
      { DriverStation.getAlliance().get() == Alliance.Red },
      robot.drive // driveRequirements - the subsystem requirements for the robot's drive train
    )
  }

  override fun periodic() {
    adstar.setStartPosition(robot.poseSubsystem.pose.translation)
  }

  fun path (goalPosition: Pose2d): Command {
    return run({
      adstar.setGoalPosition(goalPosition.translation)
    }).andThen(PrintCommand("goal updated"))
  }
}