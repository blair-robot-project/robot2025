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
import kotlin.math.floor

class AutoScorePathfinder(val robot: Robot, val endPose: Pose2d) {
  private var ADStar = LocalADStar()

  private val pathPub: StructArrayPublisher<Pose2d>
  private val velXPub: DoublePublisher
  private val velYPub: DoublePublisher
  private val velRotationPub: DoublePublisher
  private val setpointPub: BooleanPublisher
  private val rotPub: BooleanPublisher
  private val autodistancePub: BooleanPublisher
  private val pathSub: StructArraySubscriber<Pose2d>
  private val admagpub: DoublePublisher
  private val distpub: DoublePublisher
  private lateinit var path: PathPlannerPath

  private var velocityX = 0.0
  private var velocityY = 0.0
  private var rotation = 0.0
  private val timer = Timer()
  private val zeroPose = Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))
  private var inPIDDistance = false
  private var tolerance = 0.0254
  private var pidDistance = 1.0
  var atSetpoint = false

  private var trajValid = true
  private var startTime = 0.0
  private var expectedTime = 0.0
  private lateinit var trajectory: PathPlannerTrajectory
  private var pathNull = true
  private var trajectoryNull = true

  private var xPIDSpeed = 0.0
  private var yPIDSpeed = 0.0
  private val pidOffsetTime = 0.5

  var thetaController: PIDController = PIDController(7.0, 0.3, 0.1)
  private var xController = PIDController(10.0, 0.7, 0.5)
  private var yController = PIDController(10.0, 0.7, 0.5)
  private var adMag = 1.0
  private val rotTol = 0.01

  init {
    timer.restart()
    velXPub = NetworkTableInstance.getDefault().getDoubleTopic("/pathVelocityX").publish()
    velYPub = NetworkTableInstance.getDefault().getDoubleTopic("/pathVelocityY").publish()
    velRotationPub = NetworkTableInstance.getDefault().getDoubleTopic("/pathRotation").publish()
    setpointPub = NetworkTableInstance.getDefault().getBooleanTopic("/atSetpoint").publish()
    rotPub = NetworkTableInstance.getDefault().getBooleanTopic("/atRotSetpoint").publish()
    autodistancePub = NetworkTableInstance.getDefault().getBooleanTopic("/inPIDDistance").publish()
    pathPub = NetworkTableInstance.getDefault().getStructArrayTopic("/zactivePath", Pose2d.struct).publish(*arrayOf<PubSubOption>())
    distpub = NetworkTableInstance.getDefault().getDoubleTopic("/distance").publish()
    pathSub =
      NetworkTableInstance.getDefault().getStructArrayTopic("/zactivePath", Pose2d.struct)
        .subscribe(arrayOf(zeroPose, zeroPose))

    PathPlannerPath.clearCache()
    xController.reset()
    xController.reset()
    thetaController.reset()

    thetaController.enableContinuousInput(-PI, PI)
    thetaController.setTolerance(rotTol)
    thetaController.setpoint = endPose.rotation.radians
    xController.setTolerance(tolerance)
    yController.setTolerance(tolerance)
    adMag = 1.0
    admagpub = NetworkTableInstance.getDefault().getDoubleTopic("/admag").publish()

  }

  fun runSetup() {
    timer.restart()
    ADStar.setStartPosition(robot.poseSubsystem.pose.translation)
    ADStar.setGoalPosition(endPose.translation)
    velXPub.set(velocityX)
    velYPub.set(velocityY)
    velRotationPub.set(rotation)
    println("new command started")
  }

  fun pathfind() {
    val currentTime = timer.get()
    val distance = robot.poseSubsystem.pose.translation.getDistance(endPose.translation)
    println("distance: $distance")
    if (distance < pidDistance) {
      if(!inPIDDistance) {
        inPIDDistance = true
        xController.setpoint = endPose.translation.x
        yController.setpoint = endPose.translation.y
      }
//      adMag -= 0.02
//      if(adMag < 0) {
//        adMag = 0.0
//      }
      if (distance < tolerance) {
        atSetpoint = true
      }
    } else {
      inPIDDistance = false
    }
    if (!atSetpoint) {
      if (ADStar.isNewPathAvailable) {
        val newPath: PathPlannerPath? = ADStar.getCurrentPath(
          PathConstraints(
            AutoScoreCommandConstants.MAX_LINEAR_SPEED,
            AutoScoreCommandConstants.MAX_ACCEL,
            5 * 2 * Math.PI,
            RobotConstants.ROT_RATE_LIMIT
          ),
          GoalEndState(0.0, endPose.rotation)
        )
        if (newPath == null) {
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
          if (newTraj == null) {
            trajectoryNull = true
          } else {
            trajectory = newTraj
            trajectoryNull = false
          }
          if (!trajectoryNull) {
            expectedTime = trajectory.totalTimeSeconds
            pathPub.set(pathList)
            trajValid = (pathList[0] != endPose)
            startTime = currentTime
          }
        }
      }
      if (!trajectoryNull && trajValid) {
        if (inPIDDistance || currentTime - startTime + pidOffsetTime > expectedTime) {
          xPIDSpeed = xController.calculate(robot.poseSubsystem.pose.translation.x)
          yPIDSpeed = yController.calculate(robot.poseSubsystem.pose.translation.y)
          println("distance pid")
        } else {
          expectedTime = trajectory.totalTimeSeconds
          trajectory.sample(currentTime - startTime + pidOffsetTime).pose.let {
            xController.setpoint = (it.translation.x)
            yController.setpoint = (it.translation.y)
            xPIDSpeed = xController.calculate(robot.poseSubsystem.pose.translation.x)
            yPIDSpeed = yController.calculate(robot.poseSubsystem.pose.translation.y)
            println("traj pid")
          }
        }
        trajectory.sample(currentTime - startTime).fieldSpeeds.let {
          val trajSpeeds = ChassisSpeeds(
            it.vxMetersPerSecond,
            it.vyMetersPerSecond,
            it.omegaRadiansPerSecond
          )
          velocityX = trajSpeeds.vxMetersPerSecond * adMag
          velocityY = trajSpeeds.vyMetersPerSecond * adMag
          rotation = 0.0
        }
        println("pidx: $xPIDSpeed pidy: $yPIDSpeed")
      }
    } else {
      velocityX = 0.0
      velocityY = 0.0
      xPIDSpeed = 0.0
      yPIDSpeed = 0.0
      println("setpoint")
    }
    setpointPub.set(atSetpoint)
    rotPub.set(thetaController.atSetpoint())
    autodistancePub.set(inPIDDistance)
    velXPub.set(xPIDSpeed)
    velYPub.set(yPIDSpeed)
    velRotationPub.set(rotation)
    admagpub.set(adMag)
    distpub.set(distance)

    rotation = thetaController.calculate(MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians))
    if (thetaController.atSetpoint()) {
      rotation = 0.0
    }
    val fieldRelative = fromFieldRelativeSpeeds(
      ChassisSpeeds(velocityX+xPIDSpeed, velocityY+yPIDSpeed, rotation), robot.poseSubsystem.pose.rotation
    )
    robot.poseSubsystem.setPathMag(fieldRelative)
  }
}

class EmptyDrive(drive: SwerveDrive) : Command() {
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
  private val waitTime = 1.0
  private var waitTimer = waitTime
  private var reachedAD = false

  override fun initialize() {
    robot.drive.defaultCommand.cancel()
    robot.drive.defaultCommand = EmptyDrive(robot.drive)
    robot.drive.defaultCommand.schedule()
    currentCommand.runSetup()
    println("initialized")
  }

  override fun execute() {
    if (robot.poseSubsystem.pose.translation.getDistance(currentCommand.endPose.translation) < robot.poseSubsystem.autoDistance && !reachedAD) {
      println("ad reached")
      robot.superstructureManager.requestGoal(goal).schedule()
      reachedAD = true
    }
    if(currentCommand.atSetpoint && currentCommand.thetaController.atSetpoint()) {
      println("in timer")
      waitTimer -= 0.02
    } else {
      waitTimer = waitTime
      currentCommand.pathfind()
    }
  }

  override fun isFinished(): Boolean {
    if(waitTimer < 0) {
      println("finished")
      return true
    }
    return false
  }
}