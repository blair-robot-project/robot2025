package frc.team449.commands.autoscoreCommands

import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.pathfinding.RemoteADStar
import com.pathplanner.lib.trajectory.PathPlannerTrajectory
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.networktables.*
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.Robot
import frc.team449.subsystems.FieldConstants
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveDrive
import kotlin.math.PI
import kotlin.math.abs
import kotlin.properties.Delegates

class AutoScorePathfinder(private val robot: Robot, private val endPose: Pose2d, private val scoringReef: Boolean) {
  private var ADStar = RemoteADStar()

  private val velXPub: DoublePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/autoscore/pathVelocityX").publish()
  private val velYPub: DoublePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/autoscore/pathVelocityY").publish()
  private val velRotationPub: DoublePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/autoscore/pathRotation").publish()
  private val setpointPub: BooleanPublisher = NetworkTableInstance.getDefault().getBooleanTopic("/autoscore/atSetpoint").publish()
  private val rotPub: BooleanPublisher = NetworkTableInstance.getDefault().getBooleanTopic("/autoscore/atRotSetpoint").publish()
  private val autodistancePub: BooleanPublisher = NetworkTableInstance.getDefault().getBooleanTopic("/autoscore/inPIDDistance").publish()
  private val admagpub: DoublePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/autoscore/admag").publish()
  private val distpub: DoublePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/autoscore/distance").publish()
  private val atCoralRotPub: BooleanPublisher = NetworkTableInstance.getDefault().getBooleanTopic("/autoscore/atIntakeRot").publish()
  private val rotationSetpointPub: DoublePublisher = NetworkTableInstance.getDefault().getDoubleTopic("/autoscore/rotSetpoint").publish()
  private lateinit var path: PathPlannerPath

  private var velocityX = 0.0
  private var velocityY = 0.0
  private var rotation = 0.0
  private val timer = Timer()
  private var inPIDDistance = false
  private var pidDistance = 0.75
  private val tolerance = 0.035
  private val rotTol = 0.015
  private val premoveDistance = 1.0
  private val reefCenter = FieldConstants.REEF_CENTER

  var atSetpoint = false
  var atPremoveDistance = false

  private var trajValid = true
  private var startTime = 0.0
  private var expectedTime = 0.0
  private lateinit var trajectory: PathPlannerTrajectory
  private var pathNull = true
  private var trajectoryNull = true

  private var xPIDSpeed = 0.0
  private var yPIDSpeed = 0.0
  private val pidOffsetTime = 0.02

  private var thetaController = ProfiledPIDController(5.5, 0.0, 0.0, TrapezoidProfile.Constraints(AutoScoreCommandConstants.MAX_PATHFINDING_ROT_SPEED, AutoScoreCommandConstants.MAX_ROT_ACCEL))
  private var xController = ProfiledPIDController(4.0, 0.0, 0.0, TrapezoidProfile.Constraints(AutoScoreCommandConstants.MAX_PATHFINDING_LINEAR_SPEED, AutoScoreCommandConstants.MAX_ACCEL))
  private var yController = ProfiledPIDController(4.0, 0.0, 0.0, TrapezoidProfile.Constraints(AutoScoreCommandConstants.MAX_PATHFINDING_LINEAR_SPEED, AutoScoreCommandConstants.MAX_ACCEL))
  var distance: Double = 100.0
  private var ADStarPower = 0.95
  private val ADStarDecrease = 0.04
  private val pushbackMultiply = 0.35

  private val speedTol: Double = 0.075
  private val speedTolRot: Double = PI / 16
  private val ffMinRadius: Double = 0.02
  private val ffMaxRadius: Double = 0.1

  private lateinit var coralIntakeTranslation: Translation2d
  private var rotationSetpoint by Delegates.notNull<Double>()
  private var atEndRotationDistance = false
  private var finishedCoralIntakeRotation = false

  fun runSetup() {
    timer.restart()
    val currentPose = robot.poseSubsystem.pose
    ADStar.setStartPosition(currentPose.translation)
    ADStar.setGoalPosition(endPose.translation)
    velXPub.set(velocityX)
    velYPub.set(velocityY)
    velRotationPub.set(rotation)
    distance = currentPose.translation.getDistance(endPose.translation)
    if (distance < pidDistance) {
      ADStarPower = (distance / pidDistance)
    }

    val fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(
      robot.drive.currentSpeeds.vxMetersPerSecond,
      robot.drive.currentSpeeds.vyMetersPerSecond,
      robot.drive.currentSpeeds.omegaRadiansPerSecond,
      currentPose.rotation
    )

    xController.reset(
      currentPose.x,
      fieldRelative.vxMetersPerSecond
    )
    yController.reset(
      currentPose.y,
      fieldRelative.vyMetersPerSecond
    )

    thetaController.reset(
      currentPose.rotation.radians,
      fieldRelative.omegaRadiansPerSecond
    )
    thetaController.enableContinuousInput(-PI, PI)

    xController.setTolerance(0.0, speedTol)
    yController.setTolerance(0.0, speedTol)
    thetaController.setTolerance(0.0, speedTolRot)

    xController.constraints = TrapezoidProfile.Constraints(AutoScoreCommandConstants.MAX_PATHFINDING_LINEAR_SPEED, AutoScoreCommandConstants.MAX_ACCEL)
    yController.constraints = TrapezoidProfile.Constraints(AutoScoreCommandConstants.MAX_PATHFINDING_LINEAR_SPEED, AutoScoreCommandConstants.MAX_ACCEL)
    thetaController.constraints = TrapezoidProfile.Constraints(AutoScoreCommandConstants.MAX_PATHFINDING_ROT_SPEED, AutoScoreCommandConstants.MAX_ROT_ACCEL)

    coralIntakeTranslation = currentPose.translation.nearest(FieldConstants.CORAL_INTAKE_LOCATIONS)
    val closestTag = currentPose.translation.nearest(FieldConstants.APRIL_TAG_LOCATIONS)
    rotationSetpoint = closestTag.minus(coralIntakeTranslation).angle.radians
  }

  fun pathFind() {
    val currentPose = robot.poseSubsystem.pose
    ADStar.setStartPosition(currentPose.translation)
    val currentTime = timer.get()
    distance = currentPose.translation.getDistance(endPose.translation)
    val closestTag = currentPose.translation.nearest(FieldConstants.APRIL_TAG_LOCATIONS)
    if (distance < premoveDistance) {
      atPremoveDistance = true
    }
    if (distance < pidDistance) {
      inPIDDistance = true
      ADStarPower -= ADStarDecrease
      if (ADStarPower < 0) {
        ADStarPower = 0.0
      }
      if (distance < tolerance) {
        atSetpoint = true
      }
    } else {
      inPIDDistance = false
    }
    if (distance < pidDistance * 2) {
      atEndRotationDistance = true
    }

    if (!atSetpoint) {
      if (ADStar.isNewPathAvailable) {
        val newPath: PathPlannerPath? = ADStar.getCurrentPath(
          PathConstraints(
            AutoScoreCommandConstants.MAX_PATHFINDING_LINEAR_SPEED,
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
            Rotation2d(MathUtil.angleModulus(currentPose.rotation.radians)),
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
            trajValid = (pathList[0] != endPose)
            startTime = currentTime
          }
        }
      }
      if (!trajectoryNull && trajValid) {
        if (currentTime - startTime + pidOffsetTime * 2 > expectedTime) {
          xController.goal = TrapezoidProfile.State(endPose.x, 0.0)
          yController.goal = TrapezoidProfile.State(endPose.y, 0.0)
        } else {
          trajectory.sample(currentTime - startTime + pidOffsetTime).pose.let {
            xController.goal = TrapezoidProfile.State(it.x, AutoScoreCommandConstants.MAX_PATHFINDING_LINEAR_SPEED)
            yController.goal = TrapezoidProfile.State(it.y, AutoScoreCommandConstants.MAX_PATHFINDING_LINEAR_SPEED)
          }
        }
        val ffXScaler = MathUtil.clamp(
          (abs(currentPose.x - endPose.x) - ffMinRadius) / (ffMaxRadius - ffMinRadius),
          0.0,
          1.0
        )
        val ffYScaler = MathUtil.clamp(
          (abs(currentPose.y - endPose.y) - ffMinRadius) / (ffMaxRadius - ffMinRadius),
          0.0,
          1.0
        )
        xPIDSpeed = (xController.setpoint.velocity * ffXScaler + xController.calculate(currentPose.x, endPose.x))
        yPIDSpeed = (yController.setpoint.velocity * ffYScaler + yController.calculate(currentPose.y, endPose.y))
        trajectory.sample(currentTime - startTime).fieldSpeeds.let {
          val trajSpeeds = ChassisSpeeds(
            it.vxMetersPerSecond,
            it.vyMetersPerSecond,
            it.omegaRadiansPerSecond
          )
          velocityX = trajSpeeds.vxMetersPerSecond * ADStarPower
          velocityY = trajSpeeds.vyMetersPerSecond * ADStarPower
        }
      }
    }

    xPIDSpeed *= (1 - ADStarPower)
    yPIDSpeed *= (1 - ADStarPower)

    // rotation shenanigans
    if (atRotSetpoint(rotationSetpoint, 0.2) && !finishedCoralIntakeRotation) {
      finishedCoralIntakeRotation = true
    }

    rotationSetpoint = if (atEndRotationDistance) {
      endPose.rotation.radians
    } else {
      closestTag.minus(if (finishedCoralIntakeRotation) currentPose.translation else coralIntakeTranslation).angle.radians
    }

    thetaController.goal = TrapezoidProfile.State(rotationSetpoint, 0.0)

    val ffRotScaler = MathUtil.clamp(
      (abs(MathUtil.angleModulus(currentPose.rotation.radians) - rotationSetpoint)) / (PI),
      0.0,
      1.0
    )

    rotation = thetaController.setpoint.velocity * ffRotScaler +
      thetaController.calculate(currentPose.rotation.radians)

    val distanceToReef = currentPose.translation.getDistance(reefCenter)
    val reefPushbackTranslation = if (expectedTime > 0.5 && distanceToReef < 1.75 && distance > 0.6505 && scoringReef) {
      currentPose.translation.minus(reefCenter) * (distance - 0.6505) * pushbackMultiply * (if (expectedTime > 1.0) 1.0 else (expectedTime - 0.5) * 2)
    } else {
      Translation2d(0.0, 0.0)
    }

    if (atSetpoint || !finishedCoralIntakeRotation) {
      if (finishedCoralIntakeRotation && currentPose.rotation.radians == endPose.rotation.radians) {
        rotation = 0.0
      }
      robot.poseSubsystem.setPathMag(ChassisSpeeds(0.0, 0.0, rotation))
    } else {
      val fieldRelative = fromFieldRelativeSpeeds(
        ChassisSpeeds(
          MathUtil.clamp((velocityX + xPIDSpeed + reefPushbackTranslation.x), -AutoScoreCommandConstants.MAX_PATHFINDING_LINEAR_SPEED, AutoScoreCommandConstants.MAX_PATHFINDING_LINEAR_SPEED),
          MathUtil.clamp((velocityY + yPIDSpeed + reefPushbackTranslation.y), -AutoScoreCommandConstants.MAX_PATHFINDING_LINEAR_SPEED, AutoScoreCommandConstants.MAX_PATHFINDING_LINEAR_SPEED),
          MathUtil.clamp(rotation, -AutoScoreCommandConstants.MAX_PATHFINDING_ROT_SPEED, AutoScoreCommandConstants.MAX_PATHFINDING_ROT_SPEED)
        ),
        currentPose.rotation
      )
      robot.poseSubsystem.setPathMag(fieldRelative)
    }
    setpointPub.set(atSetpoint)
    rotPub.set(atRotSetpoint(endPose.rotation.radians, rotTol))
    autodistancePub.set(inPIDDistance)
    velXPub.set(xPIDSpeed)
    velYPub.set(yPIDSpeed)
    velRotationPub.set(rotation)
    admagpub.set(ADStarPower)
    distpub.set(distance)
    atCoralRotPub.set(finishedCoralIntakeRotation)
    rotationSetpointPub.set(rotationSetpoint)
  }

  private fun atRotSetpoint(setpoint: Double, tolerance: Double): Boolean {
    return abs(robot.poseSubsystem.pose.rotation.radians - setpoint) < tolerance
  }

  private fun getRotDistance(setpoint: Double): Double {
    return abs(robot.poseSubsystem.pose.rotation.radians - setpoint)
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

// goal should be a premove state
class AutoScoreWrapperCommand(
  private val robot: Robot,
  private val pathFinder: AutoScorePathfinder,
  private val premoveGoal: Command
) :
  Command() {

  private var hasPremoved = false
  private var rotSub = NetworkTableInstance.getDefault().getBooleanTopic("/autoscore/atRotSetpoint").subscribe(false)

  override fun initialize() {
    robot.drive.defaultCommand.cancel()
    robot.drive.defaultCommand = EmptyDrive(robot.drive)
    robot.drive.defaultCommand.schedule()
    pathFinder.runSetup()
  }

  override fun execute() {
    if (!hasPremoved && pathFinder.atPremoveDistance) {
      premoveGoal.schedule()
      hasPremoved = true
    }
    if (!pathFinder.atSetpoint || !rotSub.get()) {
      pathFinder.pathFind()
    }
  }

  override fun isFinished(): Boolean {
    return pathFinder.atSetpoint && rotSub.get()
  }
}
