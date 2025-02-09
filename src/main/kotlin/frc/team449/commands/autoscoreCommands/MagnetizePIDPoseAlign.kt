package frc.team449.auto.choreo

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.auto.AutoConstants
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveConstants
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.vision.PoseSubsystem
import kotlin.jvm.optionals.getOrNull
import kotlin.math.*

/**
 * Follower command so a robot go to a predefined pose
 * @param drivetrain Swerve Drivetrain to use
 * @param xController PID Controller to use for x-position error (output is next desired x velocity, not volts)
 * @param yController PID Controller to use for y-position error (output is next desired y velocity, not volts)
 * @param thetaController PID Controller to use for rotation error (output is next desired rotation velocity, not volts)
 * @param poseTol Tolerance within final pose to say it is "good enough"
 * @param timeout Maximum time to wait after trajectory has finished to get in tolerance. A very low timeout may end this command before you get in tolerance.
 */
class MagnetizePIDPoseAlign(
  private val drivetrain: SwerveDrive,
  private val poseSubsystem: PoseSubsystem,
  private val pose: Pose2d,
  private val controller: CommandXboxController,
  private val xController: PIDController = PIDController(AutoConstants.DEFAULT_X_KP, 0.0, 0.0),
  private val yController: PIDController = PIDController(AutoConstants.DEFAULT_Y_KP, 0.0, 0.0),
  private val thetaController: PIDController = PIDController(AutoConstants.DEFAULT_ROTATION_KP, 0.0, 0.0),
  poseTol: Pose2d = Pose2d(0.035, 0.035, Rotation2d(0.035)),
  private val timeout: Double = 4.2,
  private val fieldOriented: () -> Boolean = { true }
) : Command() {

  private var prevX = 0.0
  private var prevY = 0.0

  private var prevTime = 0.0

  private var dx = 0.0
  private var dy = 0.0
  private var magAcc = 0.0
  private var dt = 0.0
  private var magAccClamped = 0.0

  private var rotScaled = 0.0
  private val allianceCompensation = { if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) PI else 0.0 }
  private val directionCompensation = { if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) -1.0 else 1.0 }

  var headingLock = false

  private var rotRamp = SlewRateLimiter(RobotConstants.ROT_RATE_LIMIT)

  private val timer = Timer()

  private val rotCtrl = PIDController(
    RobotConstants.SNAP_KP,
    RobotConstants.SNAP_KI,
    RobotConstants.SNAP_KD
  )

  private var skewConstant = SwerveConstants.SKEW_CONSTANT

  private var desiredVel = doubleArrayOf(0.0, 0.0, 0.0)

  private var magnetizationPower = 5.0

  // Time in seconds until magnetization will stop if the driver is opposing magnetization
  private var magnetizationStopTime = 1.2
  private var timeUntilMagnetizationStop = magnetizationStopTime
  private var stopMagnetization = false

  init {
    addRequirements(drivetrain)

    xController.reset()
    xController.setTolerance(poseTol.x)

    yController.reset()
    yController.setTolerance(poseTol.y)

    thetaController.reset()
    thetaController.enableContinuousInput(-PI, PI)
    thetaController.setTolerance(poseTol.rotation.radians)
  }

  fun calculate(currPose: Pose2d, desState: Pose2d): ChassisSpeeds {
    val xPID = xController.calculate(currPose.x, desState.x)
    val yPID = yController.calculate(currPose.y, desState.y)
    val angPID = thetaController.calculate(currPose.rotation.radians, desState.rotation.radians) //

    return ChassisSpeeds.fromFieldRelativeSpeeds(xPID, yPID, angPID, currPose.rotation)
  }

  private fun allControllersAtReference(): Boolean {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint()
  }

  override fun initialize() {
    timer.restart()

    stopMagnetization = false

    prevX = drivetrain.currentSpeeds.vxMetersPerSecond
    prevY = drivetrain.currentSpeeds.vyMetersPerSecond
    prevTime = 0.0
    dx = 0.0
    dy = 0.0
    magAcc = 0.0
    dt = 0.0
    magAccClamped = 0.0

    rotRamp = SlewRateLimiter(
      RobotConstants.ROT_RATE_LIMIT,
      RobotConstants.NEG_ROT_RATE_LIM,
      drivetrain.currentSpeeds.omegaRadiansPerSecond
    )

    headingLock = false

    xController.reset()
    yController.reset()
    thetaController.reset()

    timer.restart()
  }

  override fun execute() {
    val currTime = timer.get()
    dt = currTime - prevTime
    prevTime = currTime

    val ctrlX = -controller.leftY
    val ctrlY = -controller.leftX

    val ctrlRadius = MathUtil.applyDeadband(
      min(sqrt(ctrlX.pow(2) + ctrlY.pow(2)), 1.0),
      RobotConstants.DRIVE_RADIUS_DEADBAND,
      1.0
    ).pow(SwerveConstants.JOYSTICK_FILTER_ORDER)

    val ctrlTheta = atan2(ctrlY, ctrlX)

    val xScaled = ctrlRadius * cos(ctrlTheta) * drivetrain.maxLinearSpeed
    val yScaled = ctrlRadius * sin(ctrlTheta) * drivetrain.maxLinearSpeed

    var xClamped = xScaled
    var yClamped = yScaled

    if (RobotConstants.USE_ACCEL_LIMIT) {
      dx = xScaled - prevX
      dy = yScaled - prevY
      magAcc = hypot(dx / dt, dy / dt)
      magAccClamped = MathUtil.clamp(magAcc, -RobotConstants.MAX_ACCEL, RobotConstants.MAX_ACCEL)

      val factor = if (magAcc == 0.0) 0.0 else magAccClamped / magAcc
      val dxClamped = dx * factor
      val dyClamped = dy * factor
      xClamped = prevX + dxClamped
      yClamped = prevY + dyClamped
    }

    prevX = xClamped
    prevY = yClamped

    rotScaled = if (!headingLock) {
      rotRamp.calculate(
        min(
          MathUtil.applyDeadband(
            abs(controller.rightX).pow(SwerveConstants.ROT_FILTER_ORDER),
            RobotConstants.ROTATION_DEADBAND,
            1.0
          ),
          1.0
        ) * -sign(controller.rightX) * drivetrain.maxRotSpeed
      )
    } else {
      MathUtil.clamp(
        rotCtrl.calculate(poseSubsystem.heading.radians),
        -RobotConstants.ALIGN_ROT_SPEED,
        RobotConstants.ALIGN_ROT_SPEED
      )
    }

    val vel = Translation2d(xClamped, yClamped)
    val controllerDesVel: ChassisSpeeds
    if (fieldOriented.invoke()) {
      /** Quick fix for the velocity skewing towards the direction of rotation
       * by rotating it with offset proportional to how much we are rotating
       **/
      vel.rotateBy(Rotation2d(-rotScaled * dt * skewConstant))

      val desVel = ChassisSpeeds.fromFieldRelativeSpeeds(
        vel.x * directionCompensation.invoke(),
        vel.y * directionCompensation.invoke(),
        rotScaled,
        poseSubsystem.heading
      )
      controllerDesVel = desVel

      desiredVel[0] = desVel.vxMetersPerSecond
      desiredVel[1] = desVel.vyMetersPerSecond
      desiredVel[2] = desVel.omegaRadiansPerSecond
    } else {
      controllerDesVel =
        ChassisSpeeds(
          vel.x,
          vel.y,
          rotScaled
        )
    }

    var angle = abs(
      atan2(
        controllerDesVel.vxMetersPerSecond,
        controllerDesVel.vyMetersPerSecond
      ) -
        atan2(pose.translation.x, pose.translation.y)
    )
    if (angle > Math.PI) {
      angle = 2 * Math.PI - angle
    }

    // 1.7... is 100 degrees in radians
    if (angle > 1.74533) {
      timeUntilMagnetizationStop -= 0.02
    } else {
      timeUntilMagnetizationStop = magnetizationStopTime
    }
    if (timeUntilMagnetizationStop <= 0) {
      stopMagnetization = true
    }

    // magnetization power is lowered if the robot is closer to the goal, which
    // makes it so the magnetization gets more powerful as the robot gets closer
    // Values need to be adjusted I haven't tested yet

    //    pid controller from     pose rn    to  pose     controller given speeds for chassis speeds
    drivetrain.set(
      calculate(poseSubsystem.pose, pose) + controllerDesVel *
        (magnetizationPower * (poseSubsystem.pose.translation.getDistance(pose.translation) / 10.0))
    )
    // constant                     pose rn                 distance to     desired pose
  }

  override fun isFinished(): Boolean {
    return allControllersAtReference() ||
      timer.hasElapsed(timeout) ||
      stopMagnetization
  }

  override fun end(interrupted: Boolean) {
    timer.stop()
    timer.reset()
    drivetrain.stop()
  }
}
