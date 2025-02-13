package frc.team449.subsystems.drive.swerve

import dev.doglog.DogLog
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.vision.PoseSubsystem
import kotlin.jvm.optionals.getOrNull
import kotlin.math.*

@Logged
class SwerveOrthogonalCommand(
  private val drive: SwerveDrive,
  private val poseEstimator: PoseSubsystem,
  private val controller: XboxController,
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

  init {
    addRequirements(drive)
    rotCtrl.enableContinuousInput(-PI, PI)
    rotCtrl.setTolerance(RobotConstants.SNAP_TO_ANGLE_TOLERANCE_RAD)
  }

  override fun initialize() {
    timer.restart()

    prevX = drive.currentSpeeds.vxMetersPerSecond
    prevY = drive.currentSpeeds.vyMetersPerSecond
    prevTime = 0.0
    dx = 0.0
    dy = 0.0
    magAcc = 0.0
    dt = 0.0
    magAccClamped = 0.0

    rotRamp = SlewRateLimiter(
      RobotConstants.ROT_RATE_LIMIT,
      RobotConstants.NEG_ROT_RATE_LIM,
      drive.currentSpeeds.omegaRadiansPerSecond
    )

    headingLock = false
  }

  fun snapToAngle(angle: Rotation2d) {
    val desAngle = MathUtil.angleModulus(angle.radians + allianceCompensation.invoke())
    rotCtrl.calculate(poseEstimator.heading.radians, desAngle)
    headingLock = true
  }

  fun exitSnapToAngle() {
    headingLock = false
  }

  fun checkSnapToAngleTolerance(): Boolean {
    return abs(rotCtrl.error) < RobotConstants.SNAP_TO_ANGLE_TOLERANCE_RAD
  }

  /** Just a helper command factory to point at a given angle and stop the heading lock once you get into tolerance
   * If you want to customize when the heading lock is lifted, use the internal snapToAngle,
   *  checkSnapToAngleTolerance, and exitSnapToAngle functions */
  fun pointAtAngleCommand(angle: Rotation2d): Command {
    return RunCommand({ snapToAngle(angle) })
      .until(::checkSnapToAngleTolerance)
      .andThen(::exitSnapToAngle)
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

    val xScaled = ctrlRadius * cos(ctrlTheta) * drive.maxLinearSpeed
    val yScaled = ctrlRadius * sin(ctrlTheta) * drive.maxLinearSpeed

    var xClamped = xScaled
    var yClamped = yScaled

    if (RobotConstants.USE_ACCEL_LIMIT) {
      dx = xScaled - prevX
      dy = yScaled - prevY
      magAcc = hypot(dx / dt, dy / dt)
      magAccClamped = MathUtil.clamp(magAcc, -drive.accel, drive.accel)

      val factor = if (magAcc == 0.0) 0.0 else magAccClamped / magAcc
      val dxClamped = dx * factor
      val dyClamped = dy * factor
      xClamped = prevX + dxClamped
      yClamped = prevY + dyClamped
    }

    prevX = xClamped
    prevY = yClamped

//    if (controller.bButtonPressed) {
//      snapToAngle(-PI / 3)
//    } else if (controller.xButtonPressed) {
//      snapToAngle(PI / 3)
//    } else if (controller.aButtonPressed) {
//      snapToAngle(0.0)
//    }

    rotScaled = if (!headingLock) {
      rotRamp.calculate(
        min(
          MathUtil.applyDeadband(
            abs(controller.rightX).pow(SwerveConstants.ROT_FILTER_ORDER),
            RobotConstants.ROTATION_DEADBAND,
            1.0
          ),
          1.0
        ) * -sign(controller.rightX) * drive.maxRotSpeed
      )
    } else {
      if (checkSnapToAngleTolerance()) headingLock = false

      MathUtil.clamp(
        rotCtrl.calculate(poseEstimator.heading.radians),
        -RobotConstants.ALIGN_ROT_SPEED,
        RobotConstants.ALIGN_ROT_SPEED
      )
    }

    val vel = Translation2d(xClamped, yClamped)

    if (fieldOriented.invoke()) {
      /** Quick fix for the velocity skewing towards the direction of rotation
       * by rotating it with offset proportional to how much we are rotating
       **/
      vel.rotateBy(Rotation2d(-rotScaled * dt * skewConstant))

      val desVel = ChassisSpeeds.fromFieldRelativeSpeeds(
        vel.x * directionCompensation.invoke(),
        vel.y * directionCompensation.invoke(),
        rotScaled,
        poseEstimator.heading
      )
      drive.set(
        desVel
      )

      desiredVel[0] = desVel.vxMetersPerSecond
      desiredVel[1] = desVel.vyMetersPerSecond
      desiredVel[2] = desVel.omegaRadiansPerSecond
    } else {
      drive.set(
        ChassisSpeeds(
          vel.x,
          vel.y,
          rotScaled
        )
      )
    }

    logData()
  }

  private fun logData() {
    DogLog.log("TeleopDrive/Controller/currX", if (abs(controller.leftY) < RobotConstants.DRIVE_RADIUS_DEADBAND) .0 else -controller.leftY)
    DogLog.log("TeleopDrive/Controller/currY", if (abs(controller.leftX) < RobotConstants.DRIVE_RADIUS_DEADBAND) .0 else -controller.leftX)
    DogLog.log("TeleopDrive/Controller/prevX", prevX)
    DogLog.log("TeleopDrive/Controller/prevY", prevY)

    DogLog.log("TeleopDrive/Delta/dx", dx)
    DogLog.log("TeleopDrive/Delta/dy", dy)
    DogLog.log("TeleopDrive/Delta/dt", dt)

    DogLog.log("TeleopDrive/Accel/Raw Accel", magAcc)
    DogLog.log("TeleopDrive/Accel/Clamped Accel", magAccClamped)

    DogLog.log("TeleopDrive/Rotation Control/In Heading Lock", headingLock)
    DogLog.log("TeleopDrive/Rotation Control/In Snap to Angle Tolerance", checkSnapToAngleTolerance())
  }
}
