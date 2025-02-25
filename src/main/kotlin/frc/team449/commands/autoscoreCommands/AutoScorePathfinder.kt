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

class AutoScorePathfinder(val robot: Robot, private val endPose: Pose2d) : Command() {
  private var ADStar = LocalADStar()

  private var pathPub: StructArrayPublisher<Pose2d>
  private var velXPub: DoublePublisher
  private var velYPub: DoublePublisher
  private var velRotationPub: DoublePublisher
  private var setpointPub: BooleanPublisher
  private var rotPub: BooleanPublisher
  private var autodistancePub: BooleanPublisher
  private var pathSub: StructArraySubscriber<Pose2d>
  private var admagpub: DoublePublisher
  private lateinit var path: PathPlannerPath

  private var velocityX = 0.0
  private var velocityY = 0.0
  private var rotation = 0.0
  private val timer = Timer()
  private val zeroPose = Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))
  var inPIDDistance = false
  private var tolerance = 0.0254
  private var pidDistance = 0.25
  var atSetpoint = false

  private var trajValid = true
  private var startTime = 0.0
  private var expectedTime = 0.0
  private lateinit var trajectory: PathPlannerTrajectory
  private var pathNull = true
  private var trajectoryNull = true

  private var pathIndex = 0
  private var alongPath = 0.0
  private var prevPose: Pose2d? = robot.poseSubsystem.pose
  private var nextPose: Pose2d? = Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))
  private var xPIDSpeed = 0.0
  private var yPIDSpeed = 0.0

  var thetaController: PIDController = PIDController(7.0, 0.3, 0.1)
  private var xController = PIDController(4.5, 0.1, 0.0)
  private var yController = PIDController(4.5, 0.1, 0.0)
  private var adMag = 1.0
  private val rotTol = 0.01
  var wrapperDone = false

  init {
    timer.restart()
    velXPub = NetworkTableInstance.getDefault().getDoubleTopic("/pathVelocityX").publish()
    velYPub = NetworkTableInstance.getDefault().getDoubleTopic("/pathVelocityY").publish()
    velRotationPub = NetworkTableInstance.getDefault().getDoubleTopic("/pathRotation").publish()
    setpointPub = NetworkTableInstance.getDefault().getBooleanTopic("/atSetpoint").publish()
    rotPub = NetworkTableInstance.getDefault().getBooleanTopic("/atRotSetpoint").publish()
    autodistancePub = NetworkTableInstance.getDefault().getBooleanTopic("/inPIDDistance").publish()
    pathPub = NetworkTableInstance.getDefault().getStructArrayTopic("/zactivePath", Pose2d.struct).publish(*arrayOf<PubSubOption>())

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

  private fun resetVelocities() {
    velocityX = 0.0
    velocityY = 0.0
    rotation = 0.0
    velXPub.set(velocityX)
    velYPub.set(velocityY)
    velRotationPub.set(rotation)
  }

  override fun initialize() {
    timer.restart()
    resetVelocities()
    ADStar.setStartPosition(robot.poseSubsystem.pose.translation)
    ADStar.setGoalPosition(endPose.translation)
    println("new command started")
  }

  override fun execute() {
    val currentTime = timer.get()
    ADStar.setStartPosition(robot.poseSubsystem.pose.translation)
    if (!atSetpoint) {
      val distance = robot.poseSubsystem.pose.translation.getDistance(endPose.translation)
      if (distance < pidDistance) {
        inPIDDistance = true
        adMag = 0.0
        xController.setpoint = endPose.translation.x
        yController.setpoint = endPose.translation.y
        if (distance < tolerance) {
          atSetpoint = true
        }
      }
      if (ADStar.isNewPathAvailable) {
        val newPath: PathPlannerPath? = ADStar.getCurrentPath(
          PathConstraints(
            AutoScoreCommandConstants.MAX_LINEAR_SPEED,
            AutoScoreCommandConstants.MAX_ACCEL,
            5 * 2 * Math.PI,
            RobotConstants.ROT_RATE_LIMIT
          ),
          GoalEndState(0.1, endPose.rotation)
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
        if (inPIDDistance) {
          xPIDSpeed = xController.calculate(robot.poseSubsystem.pose.translation.x)
          yPIDSpeed = yController.calculate(robot.poseSubsystem.pose.translation.y)
        } else {
          expectedTime = trajectory.totalTimeSeconds
          alongPath = (timer.get() - startTime) / expectedTime
          val numSections = pathSub.get()!!.size - 1
          pathIndex = (floor(alongPath * numSections)).toInt() + 1
          if (pathIndex < numSections - 1) {
            prevPose = pathSub.get()!![pathIndex - 1]
            nextPose = pathSub.get()!![pathIndex] //
            if (prevPose != null && nextPose != null) {
              xPIDSpeed = xController.calculate(robot.poseSubsystem.pose.x, nextPose!!.x)
              yPIDSpeed = yController.calculate(robot.poseSubsystem.pose.y, nextPose!!.y)
            } else {
              xPIDSpeed = 0.0
              yPIDSpeed = 0.0
            }
          }
          expectedTime = trajectory.totalTimeSeconds
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

          velocityX += xPIDSpeed
          velocityY += yPIDSpeed

          rotation = thetaController.calculate(MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians))
          if (thetaController.atSetpoint()) {
            rotation = 0.0
          }

          val fieldRelative = fromFieldRelativeSpeeds(ChassisSpeeds(velocityX, velocityY, rotation), robot.poseSubsystem.pose.rotation)
          robot.poseSubsystem.setPathMag(fieldRelative)
        }
      } else {
        rotation = thetaController.calculate(MathUtil.angleModulus(robot.poseSubsystem.pose.rotation.radians))
        if (thetaController.atSetpoint()) {
          rotation = 0.0
        }

        robot.drive.set(ChassisSpeeds(0.0, 0.0, rotation))
      }
      setpointPub.set(atSetpoint)
      rotPub.set(thetaController.atSetpoint())
      autodistancePub.set(inPIDDistance)
      velXPub.set(velocityX)
      velYPub.set(velocityY)
      velRotationPub.set(rotation)
      admagpub.set(adMag)
    }

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
    robot.drive.defaultCommand.initialize()
    currentCommand.initialize()
    waitTimer = waitTime
    reachedAD = false
  }

  override fun execute() {
    currentCommand.execute()
  }

  override fun isFinished(): Boolean {
    if(currentCommand.atSetpoint && currentCommand.thetaController.atSetpoint()) {
      if(waitTimer < 0) {
        currentCommand.wrapperDone = true
        currentCommand.cancel()
        this.cancel()
        return true
      }
      waitTimer -= 0.02
    } else {
      waitTimer = waitTime
    }
    if (currentCommand.inPIDDistance && !reachedAD) {
      println("ad reached")
      robot.superstructureManager.requestGoal(goal).schedule()
      reachedAD = true
    }
    return false
  }

  override fun end(interrupted: Boolean) {
    currentCommand.end(interrupted)
    super.end(interrupted)
  }
}