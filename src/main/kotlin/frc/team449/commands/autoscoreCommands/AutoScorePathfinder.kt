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
import frc.team449.Robot
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.superstructure.SuperstructureGoal
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.hypot

class AutoScorePathfinder(val robot: Robot, private val endPose: Pose2d) : Command() {
  var ADStar = LocalADStar()
  private var pathPub: StructArrayPublisher<Pose2d>
  private var velXPub: DoublePublisher
  private var velYPub: DoublePublisher
  private var velRotationPub: DoublePublisher
  private var setpointPub: BooleanPublisher
  private var autodistancePub: BooleanPublisher
  private var pathSub: StructArraySubscriber<Pose2d>
  private var velXSub: DoubleSubscriber
  private var velYSub: DoubleSubscriber
  private var setpointSub: BooleanSubscriber
  private var autodistanceSub: BooleanSubscriber
  private var velRotationSub: DoubleSubscriber
  private lateinit var path: PathPlannerPath
  private var velocityX = 0.0
  private var velocityY = 0.0
  private var rotation = 0.0
  private val timer = Timer()
  private val zeroPose = Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))
  var inAutodistanceTolerance = false
  private var tolerance = /*0.0254*/ 0.11
  private var autodistanceTolerance = robot.poseSubsystem.autoDistance
  var atSetpoint = false
  private var trajValid = true
  private var startTime = 0.0
  private var expectedTime = 0.0
  private lateinit var trajectory: PathPlannerTrajectory
  private var pathNull = true
  private var trajectoryNull = true
  private var thetaController: PIDController = PIDController(3.5, 0.0, 0.0)
  private var atRotSetpoint = false
  private var rotTol = 0.015

  init {
    timer.restart()
    velXPub = NetworkTableInstance.getDefault().getDoubleTopic("/pathVelocityX").publish()
    velYPub = NetworkTableInstance.getDefault().getDoubleTopic("/pathVelocityY").publish()
    velRotationPub = NetworkTableInstance.getDefault().getDoubleTopic("/pathRotation").publish()
    setpointPub = NetworkTableInstance.getDefault().getBooleanTopic("/atSetpoint").publish()
    autodistancePub = NetworkTableInstance.getDefault().getBooleanTopic("/inAutoDistance").publish()
    pathPub = NetworkTableInstance.getDefault().getStructArrayTopic("/zactivePath", Pose2d.struct).publish(*arrayOf<PubSubOption>())

    velXSub = NetworkTableInstance.getDefault().getDoubleTopic("/pathVelocityX").subscribe(0.0)
    velYSub = NetworkTableInstance.getDefault().getDoubleTopic("/pathVelocityY").subscribe(0.0)
    velRotationSub = NetworkTableInstance.getDefault().getDoubleTopic("/pathRotation").subscribe(0.0)
    setpointSub = NetworkTableInstance.getDefault().getBooleanTopic("/atSetpoint").subscribe(false)
    autodistanceSub = NetworkTableInstance.getDefault().getBooleanTopic("/inAutoDistance").subscribe(false)
    pathSub =
      NetworkTableInstance.getDefault().getStructArrayTopic("/zactivePath", Pose2d.struct)
        .subscribe(arrayOf(zeroPose, zeroPose))
    PathPlannerPath.clearCache()
    thetaController.enableContinuousInput(-PI, PI)
  }

  private fun resetVelocities() {
    velocityX = 0.0
    velocityY = 0.0
    rotation = 0.0
    velXPub.set(velocityX)
    velYPub.set(velocityY)
    velRotationPub.set(rotation)
  }

  private fun resetVars() {
    resetVelocities()
    inAutodistanceTolerance = false
    atSetpoint = false
    trajValid = true
    startTime = 0.0
    expectedTime = 0.0
    pathNull = true
    trajectoryNull = true
    atRotSetpoint = false
  }

  override fun initialize() {
    resetVars()
    ADStar.setStartPosition(robot.poseSubsystem.pose.translation)
    ADStar.setGoalPosition(endPose.translation)
    println("new command started")
  }

  override fun execute() {
    println("x displacement: ${robot.poseSubsystem.pose.translation.x-endPose.translation.x}")
    println("y displacement: ${robot.poseSubsystem.pose.translation.y-endPose.translation.y}")//
//    println("trajValid: $trajValid")
//    println("pathNull: $pathNull")
//    println("trajectoryNull: $trajectoryNull")
//    println("end pose rotation: ${MathUtil.angleModulus(endPose.rotation.radians)}")
    if (MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians)>MathUtil.angleModulus(endPose.rotation.radians-rotTol) && MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians)<MathUtil.angleModulus(endPose.rotation.radians+rotTol)){
      atRotSetpoint = true
//      println("at rot setpoint")
    }
    else {
      atRotSetpoint = false
    }
    val currentTime = timer.get()
    if(!atSetpoint) {
      atSetpoint = false
      inAutodistanceTolerance = false
      if(abs(robot.poseSubsystem.pose.translation.x - endPose.translation.x) < autodistanceTolerance) {
        if(abs(robot.poseSubsystem.pose.translation.y - endPose.translation.y) < autodistanceTolerance) {
          inAutodistanceTolerance = true
          println("inAutodistanceTolerance")
          if(abs(robot.poseSubsystem.pose.translation.x - endPose.translation.x) < tolerance) {
            if(abs(robot.poseSubsystem.pose.translation.y - endPose.translation.y) < tolerance) {
//              println("at setpoint")
              println("should be at setpoint")
              atSetpoint = true
            } } } }
      if (ADStar.isNewPathAvailable) {
        ADStar.setStartPosition(robot.poseSubsystem.pose.translation)
        val newPath: PathPlannerPath? = ADStar.getCurrentPath(
          PathConstraints(
            AutoScoreCommandConstants.MAX_LINEAR_SPEED,
            AutoScoreCommandConstants.MAX_ACCEL,
            5*2*Math.PI,
            RobotConstants.ROT_RATE_LIMIT
          ),
          GoalEndState(0.0, endPose.rotation)
        )
        if(newPath == null) {
          pathNull = true
        } else {
          path = newPath
          pathNull = false
        }
        if (!pathNull) {
          val pathList = path.pathPoses.toTypedArray<Pose2d>()
          val newTraj: PathPlannerTrajectory? = path.generateTrajectory(
            robot.drive.currentSpeeds,
            Rotation2d(MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians)),
            RobotConfig.fromGUISettings()
          )
          if(newTraj == null) {
            trajectoryNull = true
          } else {
            trajectory = newTraj
            trajectoryNull = false
          }
          if(!trajectoryNull) {
            expectedTime = trajectory.totalTimeSeconds
            pathPub.set(pathList)
            trajValid = pathList[0] !=
              zeroPose
            startTime = currentTime //
          }
        } else {
          if(inAutodistanceTolerance) {
//            println("at setpoint")
            println("should be at setpoint")
            atSetpoint = true
          }
        }
      }
      if(!trajectoryNull && trajValid) {
        expectedTime = trajectory.totalTimeSeconds
        trajectory.sample(currentTime - startTime).fieldSpeeds.let {
          val trajSpeeds = ChassisSpeeds(
            it.vxMetersPerSecond,
            it.vyMetersPerSecond,
            it.omegaRadiansPerSecond
          )
          velocityX = trajSpeeds.vxMetersPerSecond
          velocityY = trajSpeeds.vyMetersPerSecond
          rotation = 0.0
        }
        if (!(atRotSetpoint)) {
//          println("calculated rot speed")
//          println("rot now: ${MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians)}")
//          println("des rot: ${MathUtil.angleModulus(endPose.rotation.radians)}")
          rotation = thetaController.calculate(MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians), MathUtil.angleModulus(endPose.rotation.radians))
        }
        else {
//          println("rot speed 0, at setpoint")
//          println("rot now: ${MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians)}")
//          println("des rot: ${MathUtil.angleModulus(endPose.rotation.radians)}")
          rotation = 0.0
        }
//        println("rot speed: $rotation")
        if (!(robot.driveController.rightX>-0.05 && robot.driveController.rightX<0.05)) {
          rotation = ((rotation + robot.driveController.rightX * 2.0*(hypot(endPose.x-robot.poseSubsystem.pose.x, endPose.y-robot.poseSubsystem.pose.y))) / 2)
        }
        val fieldRelative = fromFieldRelativeSpeeds(ChassisSpeeds(velocityX, velocityY, rotation),robot.poseSubsystem.pose.rotation)
        robot.poseSubsystem.setPathMag(fieldRelative)
        trajValid = ((pathSub.get()[0]) != endPose)
      } else {
        expectedTime = 0.0
      }
    } else {
      robot.drive.set(ChassisSpeeds(0.0, 0.0, 0.0))
    }
    setpointPub.set(atSetpoint)
    autodistancePub.set(inAutodistanceTolerance)
    velXPub.set(velocityX)
    velYPub.set(velocityY)
    velRotationPub.set(rotation)
  }

  override fun isFinished(): Boolean {
    return inAutodistanceTolerance
  }

  override fun end(interrupted: Boolean) {
    resetVars()
  }
}

class EmptyDrive(private val drive: SwerveDrive) : Command() {
  init {
    addRequirements(drive)
  }
  override fun isFinished(): Boolean {
    return true
  }
}
//goal should be a premove state
class AutoscoreWrapperCommand(
  val robot: Robot,
  command: AutoScorePathfinder,
  private val goal: SuperstructureGoal.SuperstructureState,
)
  : Command() {

  private val currentCommand = command
  private var setpointReached = false
  private var adReached = false

//  init {
//    addRequirements(robot.drive)
//    addRequirements(robot.poseSubsystem)
//  }
  override fun initialize() {
    if (currentCommand.isScheduled) {
      currentCommand.cancel()
      this.cancel()
    }
    currentCommand.addRequirements(robot.drive)
    currentCommand.addRequirements(robot.poseSubsystem)
    robot.drive.defaultCommand.cancel()
    robot.drive.defaultCommand = EmptyDrive(robot.drive)
    robot.drive.defaultCommand.initialize()
    currentCommand.initialize()
    setpointReached = false
    adReached = false//
  }

  override fun execute() {
    currentCommand.execute()
  }

  private fun resetAndEndCommand() {
    currentCommand.end(true)
    currentCommand.cancel()
    setpointReached = false
    adReached = false
    this.end(true)
    this.cancel()//
  }

  override fun isFinished(): Boolean {
    if (currentCommand.atSetpoint) {
      println("autoscore command finished")
      resetAndEndCommand()
      return true
    }

      if (currentCommand.inAutodistanceTolerance && !adReached) {
        robot.superstructureManager.requestGoal(goal)
      }
      return false

  }

  override fun end(interrupted: Boolean) {
    currentCommand.end(interrupted)
    robot.drive.defaultCommand = robot.driveCommand
  }
}

class getToRot(
  val robot: Robot,
  private val endPose: Pose2d,
  private var thetaController: PIDController = PIDController(3.5, 0.0, 0.0)
) : Command() {
//  var atRotSetpoint = ((MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians) > MathUtil.angleModulus(endPose.rotation.radians - 0.07) && MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians) < MathUtil.angleModulus(
//    endPose.rotation.radians + 0.07)))
  private var atRotSetpoint = false

  override fun initialize() {
    atRotSetpoint = false
  }

  override fun execute() {
    atRotSetpoint = ((MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians) > MathUtil.angleModulus(endPose.rotation.radians - 0.07) && MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians) < MathUtil.angleModulus(
      endPose.rotation.radians + 0.07)))


    println("getting to rot")
    if (!atRotSetpoint) {
      println("not at rot setpoint")
      val rotation = thetaController.calculate(MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians), MathUtil.angleModulus(endPose.rotation.radians))
      robot.drive.set(ChassisSpeeds(0.0, 0.0, rotation))
    }
    else {
      println("at rot setpoint")
    }
  }

  override fun isFinished(): Boolean {
    return atRotSetpoint
  }

  override fun end(interrupted: Boolean) {
    robot.drive.set(ChassisSpeeds(0.0,0.0,0.0))
  }
  }