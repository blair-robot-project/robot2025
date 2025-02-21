package frc.team449.commands.autoscoreCommands

import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.pathfinding.LocalADStar
import com.pathplanner.lib.trajectory.PathPlannerTrajectory
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds
import edu.wpi.first.networktables.*
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Robot
import frc.team449.subsystems.RobotConstants

class PathMag(val robot: Robot): SubsystemBase() {
  var adStar = LocalADStar()
  private var pathPub: StructArrayPublisher<Pose2d>? = null
  private var pathSub: StructArraySubscriber<Pose2d>? = null
  private var path: PathPlannerPath? = null
  private val timer = Timer()
  private var expectedTime: Double? = 0.0
  private var startTime = 0.0
  private var currentSpeed: ChassisSpeeds = ChassisSpeeds(0.0, 0.0, 0.0)
  private var pathRunning = false
  private var pathValid = false
  private var pathRotDone = true
  private var goalPos: Pose2d = Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))
  private var trajectory: PathPlannerTrajectory? = null
  private var thetaController: PIDController = PIDController(4.0, 0.0,0.0)
  private var desRot = (0.0)
  private var tolerance = 0.1
//
  init {
    timer.restart()
    pathPub = NetworkTableInstance.getDefault().getStructArrayTopic("/ADStarPath", Pose2d.struct).publish(*arrayOf<PubSubOption>())
    pathSub =
      NetworkTableInstance.getDefault().getStructArrayTopic("/ADStarPath", Pose2d.struct)
        .subscribe(arrayOf(Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0)), Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))))
  }

  override fun periodic() {
    if (robot.poseSubsystem.pose.translation.x >= goalPos.translation.x - tolerance &&  robot.poseSubsystem.pose.translation.x <= goalPos.translation.x + tolerance
      && robot.poseSubsystem.pose.translation.y >= goalPos.translation.y - tolerance && robot.poseSubsystem.pose.translation.y <= goalPos.translation.y + tolerance
      ) {
      println("at translation setpoint!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    }
    println("path running: $pathRunning")
    println("path valid: $pathValid")
    println("desired rot value: ${goalPos.rotation.radians}")
    println("rot now: ${MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians)}")
    desRot = thetaController.calculate(MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians), MathUtil.angleModulus(goalPos.rotation.radians))
    //println("rotation: ${robot.poseSubsystem.pose.rotation}")
    if (adStar.isNewPathAvailable) {
      println("new path")

      path = adStar.getCurrentPath(
        PathConstraints(
          AutoScoreCommandConstants.MAX_LINEAR_SPEED,
          AutoScoreCommandConstants.MAX_ACCEL,
          5*2*Math.PI,
          RobotConstants.ROT_RATE_LIMIT
        ),
        GoalEndState(0.5, goalPos.rotation)
        //GoalEndState(0.0, Rotation2d(0.0))
      )
      if (path != null) {
        println("new path, not null")
        println("new trajectory")
        trajectory = path!!.generateTrajectory(
          robot.drive.currentSpeeds,
          Rotation2d(MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians)),
          //Rotation2d(0.0),
          RobotConfig.fromGUISettings()
        )
        expectedTime = trajectory!!.totalTimeSeconds
        pathPub?.set(path!!.pathPoses.toTypedArray<Pose2d>())
        pathRunning = (robot.poseSubsystem.pose != goalPos)
        pathValid = ((pathSub?.get()?.get(0) ?: Pose2d()) != Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0)))
        startTime = timer.get() //
      }
      else{
        println("new path, null ")
        if (!(((MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians) >= goalPos.rotation.radians-tolerance) && (MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians) <= goalPos.rotation.radians +tolerance)))){
          pathRotDone = false
          //println("keep moving to setpoint 1")
          //println("rot just finished: ")
          //robot.poseSubsystem.pathfindingMagnetize(fromFieldRelativeSpeeds(ChassisSpeeds(robot.drive.currentSpeeds.vyMetersPerSecond, robot.drive.currentSpeeds.vxMetersPerSecond, desRot), robot.poseSubsystem.pose.rotation))
        }
        pathRunning=false
        pathValid=false
      }
    }
    if (!pathRotDone) {
      println("keep moving to setpoint")
      println("desired rotation speed: $desRot")
      println("if path rot not done set: ")
      robot.poseSubsystem.setPathMag(fromFieldRelativeSpeeds(ChassisSpeeds(robot.drive.currentSpeeds.vyMetersPerSecond, robot.drive.currentSpeeds.vxMetersPerSecond, desRot), robot.poseSubsystem.pose.rotation))
      //robot.drive.set((ChassisSpeeds(robot.drive.currentSpeeds.vyMetersPerSecond, robot.drive.currentSpeeds.vxMetersPerSecond, desRot)))
      if (((MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians) >= goalPos.rotation.radians-tolerance) && (MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians) <= goalPos.rotation.radians +tolerance))){
        println("ok now it's at the setpoint")
        pathRotDone = true
      }
    }
    if (pathRunning && pathValid) {
      if (trajectory != null) {
        expectedTime = trajectory?.totalTimeSeconds
        if (timer.get() - startTime < expectedTime!!) {
          if (((MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians) >= goalPos.rotation.radians-tolerance) && (MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians) <= goalPos.rotation.radians +tolerance))){
            currentSpeed = trajectory?.sample(timer.get() - startTime)?.fieldSpeeds?.let {
              ChassisSpeeds(
                it.vxMetersPerSecond,
                it.vyMetersPerSecond,
                //it.omegaRadiansPerSecond)
              //desRot
              0.0)
            } ?: robot.drive.currentSpeeds
          }
          else {
            println("not at setpointttttttttttttttttttttttttttttttttttttttttttttttt")
            currentSpeed = trajectory?.sample(timer.get() - startTime)?.fieldSpeeds?.let {
              ChassisSpeeds(
                it.vxMetersPerSecond,
                it.vyMetersPerSecond,
                //it.omegaRadiansPerSecond
                desRot
                //robot.driveController.rightX*3.0
              )
            } ?: robot.drive.currentSpeeds
          }
          println("\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t desired rotation speed: $desRot")
          println("time since start: ${timer.get() - startTime}")
          println("speed now: $currentSpeed")
          println("normal set for running translation path: ")
          val setCommand = RunCommand({
            robot.poseSubsystem.setPathMag(fromFieldRelativeSpeeds(currentSpeed,robot.poseSubsystem.pose.rotation))
          }).withTimeout(0.02)
          setCommand.addRequirements(robot.drive)
          setCommand.addRequirements(robot.poseSubsystem)
          setCommand.schedule()
          pathRunning = (robot.poseSubsystem.pose != goalPos)
          pathValid = ((pathSub?.get()?.get(0) ?: Pose2d()) != Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0)))
          println()
        } else {
          //
//          SimpleReefAlign(robot.drive, robot.poseSubsystem).schedule()
          expectedTime = 0.0
        }
        adStar.setStartPosition(robot.poseSubsystem.pose.translation)
      }
    }
  }
  fun getPath(goalPosition: Pose2d): Command {
    return runOnce {
      goalPos = goalPosition
      startTime = timer.get() //
      adStar.setStartPosition(robot.poseSubsystem.pose.translation)
      adStar.setGoalPosition(goalPosition.translation)
    }.andThen(PrintCommand("\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t goal updated")) //

  }
}
