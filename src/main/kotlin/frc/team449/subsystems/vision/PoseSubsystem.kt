package frc.team449.subsystems.vision

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.auto.AutoConstants
import frc.team449.control.vision.ApriltagCamera
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveConstants
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.drive.swerve.SwerveSim
import frc.team449.subsystems.vision.interpolation.InterpolatedVision
import frc.team449.system.AHRS
import kotlin.jvm.optionals.getOrNull
import kotlin.math.*
import kotlin.time.times

class PoseSubsystem(
  private val ahrs: AHRS,
  private val cameras: List<ApriltagCamera> = mutableListOf(),
  private val drive: SwerveDrive,
  private val field: Field2d,
  val controller: CommandXboxController,
  private val xController: PIDController = PIDController(AutoConstants.DEFAULT_X_KP, 0.0, 0.0),
  private val yController: PIDController = PIDController(AutoConstants.DEFAULT_Y_KP, 0.0, 0.0),
  private val thetaController: PIDController = PIDController(AutoConstants.DEFAULT_ROTATION_KP, 0.0, 0.0),
  poseTol: Pose2d = Pose2d(0.035, 0.035, Rotation2d(0.035)),
  private val timeout: Double = 4.2,
  private val fieldOriented: () -> Boolean = { true },
) : SubsystemBase() {

  /** magnetize stuff */
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
//  lateinit var endPose: Pose2d
  var autoscoreCommandPose = Pose2d(0.0, 0.0,Rotation2d(0.0))

  private val rotCtrl = PIDController(
    RobotConstants.SNAP_KP,
    RobotConstants.SNAP_KI,
    RobotConstants.SNAP_KD
  )

  private var skewConstant = SwerveConstants.SKEW_CONSTANT
  private var desiredVel = doubleArrayOf(0.0, 0.0, 0.0)
  private var magnetizationPower = 50.0
  // Time in seconds until magnetization will stop if the driver is opposing magnetization
  private var magnetizationStopTime = 1.2
  private var timeUntilMagnetizationStop = magnetizationStopTime
  private var stopMagnetization = false
  private var resistanceAngle = 100.0
  //placeholder extremely big number
  private var lastDistance = 1000.0
  private var autoDistance = 0.5

  //edem mag vars
  private var currentMagPower = 8.0
  private var magMultiply = 1.015
  private val magIncConstant = 0.001
  private var magDecConstant = 0.0003
  private var stopControllerInput = false


  init {
    xController.reset()
    xController.setTolerance(poseTol.x)

    yController.reset()
    yController.setTolerance(poseTol.y)

    thetaController.reset()
    thetaController.enableContinuousInput(-PI, PI)
    thetaController.setTolerance(poseTol.rotation.radians)

    timer.restart()

    stopMagnetization = false

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

    xController.reset()
    yController.reset()
    thetaController.reset()

    timer.restart()
  }

  private fun calculate(currPose: Pose2d, desState: Pose2d): ChassisSpeeds {
    val xPID = xController.calculate(currPose.x, desState.x)
    val yPID = yController.calculate(currPose.y, desState.y)
    val angPID = thetaController.calculate(currPose.rotation.radians, desState.rotation.radians)

    return ChassisSpeeds.fromFieldRelativeSpeeds(xPID, yPID, angPID, currPose.rotation)
  }

  private fun allControllersAtReference(): Boolean {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint()
  }

  fun pathfindingMagnetize(desVel: ChassisSpeeds) {
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
        ) * -sign(controller.rightX) * drive.maxRotSpeed
      )
    } else {
      MathUtil.clamp(
        rotCtrl.calculate(this.heading.radians),
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

//      val desVel = ChassisSpeeds.fromFieldRelativeSpeeds(
//        vel.x * directionCompensation.invoke(),
//        vel.y * directionCompensation.invoke(),
//        rotScaled,
//        this.heading
//      )
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

    var driverResistance = abs(
      atan2(controllerDesVel.vxMetersPerSecond,
        controllerDesVel.vyMetersPerSecond) -
        atan2(pose.translation.x, pose.translation.y)
    )
    if (driverResistance > Math.PI) {
      driverResistance = 2 * Math.PI - driverResistance
    }

    // 1.7... is 100 degrees in radians
    if (driverResistance > Units.degreesToRadians(resistanceAngle)) {
      timeUntilMagnetizationStop -= 0.02
    } else {
      timeUntilMagnetizationStop = magnetizationStopTime
    }
    if (timeUntilMagnetizationStop <= 0) {
      stopMagnetization = true
    }

    // magnetization power is lowered if the robot is closer to the goal, which
//    // makes it so the magnetization gets more powerful as the robot gets closer
//    // Values need to be adjusted I haven't tested yet
//    println("pose: ${pose.translation} endPose: ${endPose.translation}")
//
//    //    pid controller from     pose rn    to  pose     controller given speeds for chassis speeds
//    drive.set(calculate(this.pose, endPose) + controllerDesVel *
//      (magnetizationPower * (this.pose.translation.getDistance(endPose.translation) / 10.0)))
//    // constant                     pose rn                 distance to     desired pose
  }

  fun edemPathMag(desVel: ChassisSpeeds) {
    val currTime = timer.get()
    dt = currTime - prevTime
    prevTime = currTime
    val distance = pose.translation.getDistance(autoscoreCommandPose.translation)
    if(distance <= autoDistance) {
      currentMagPower = 8.0
      magMultiply = 1.015
      magDecConstant = 0.0003
      drive.set(desVel)
    } else {

      val ctrlX = if (abs(controller.leftY) < RobotConstants.DRIVE_RADIUS_DEADBAND) .0 else controller.leftY
      val ctrlY = if (abs(controller.leftX) < RobotConstants.DRIVE_RADIUS_DEADBAND) .0 else controller.leftX

      val controllerAngle = atan2(ctrlY, ctrlX)
      var controllerSpeeds = ChassisSpeeds(ctrlX, ctrlY, controllerAngle)
      controllerSpeeds *= currentMagPower
      //des vel x and y switched
      //des vel left is negative x
      //des vel right is negative y
      //des vel up is positive x
      //des vel down is positive y
      //controller x and y work normally
      //this means:
      //if des vel x is negative : des vel moving left
      //needs to fight against controller x
      //if des vel x is positive: des vel moving up
      //needs to fight against controller y
      //if des vel y is negative : des vel moving right
      //needs to fight against controller x
      //if des vel y is positive : des vel moving down
      //needs to fight against controller y

      val combinedChassisSpeeds = ChassisSpeeds()
      if(desVel.vxMetersPerSecond < 0) {
        combinedChassisSpeeds.vxMetersPerSecond = desVel.vxMetersPerSecond + controllerSpeeds.vxMetersPerSecond
      } else {
        combinedChassisSpeeds.vxMetersPerSecond = desVel.vxMetersPerSecond + controllerSpeeds.vyMetersPerSecond
      }
      if(desVel.vyMetersPerSecond < 0) {
        combinedChassisSpeeds.vyMetersPerSecond = desVel.vyMetersPerSecond + controllerSpeeds.vxMetersPerSecond
      } else {
        combinedChassisSpeeds.vyMetersPerSecond = desVel.vyMetersPerSecond + controllerSpeeds.vyMetersPerSecond
      }
      combinedChassisSpeeds.omegaRadiansPerSecond = desVel.omegaRadiansPerSecond

      println("controller speeds: $controllerSpeeds")
      drive.set(combinedChassisSpeeds)

      if(distance > lastDistance) {
        magMultiply += magIncConstant
      } else if(distance < lastDistance) {
        magMultiply -= magDecConstant
        magDecConstant += 0.000002
      }
      currentMagPower *= magMultiply
    }
    lastDistance = distance
  }

  private val isReal = RobotBase.isReal()

  private val poseEstimator = SwerveDrivePoseEstimator(
    drive.kinematics,
    ahrs.heading,
    drive.getPositions(),
    RobotConstants.INITIAL_POSE,
    VisionConstants.ENCODER_TRUST,
    VisionConstants.MULTI_TAG_TRUST
  )

  var heading: Rotation2d
    get() = Rotation2d(MathUtil.angleModulus(this.pose.rotation.radians))
    set(value) {
      this.pose = Pose2d(Translation2d(this.pose.x, this.pose.y), value)
    }

  /** Vision statistics */
  private val numTargets = DoubleArray(cameras.size)
  private val tagDistance = DoubleArray(cameras.size)
  private val avgAmbiguity = DoubleArray(cameras.size)
  private val heightError = DoubleArray(cameras.size)
  private val usedVision = BooleanArray(cameras.size)
  private val usedVisionSights = LongArray(cameras.size)
  private val rejectedVisionSights = LongArray(cameras.size)

  var enableVisionFusion = true

  /** Current estimated vision pose */
  var visionPose = DoubleArray(cameras.size * 3)

  /** The measured pitch of the robot from the gyro sensor. */
  val pitch: Rotation2d
    get() = Rotation2d(MathUtil.angleModulus(ahrs.pitch.radians))

  /** The measured roll of the robot from the gyro sensor. */
  val roll: Rotation2d
    get() = Rotation2d(MathUtil.angleModulus(ahrs.roll.radians))

  var oldPose: Pose2d = Pose2d()

  /** The (x, y, theta) position of the robot on the field. */
  var pose: Pose2d
    get() = this.poseEstimator.estimatedPosition
    set(value) {
      this.poseEstimator.resetPosition(
        ahrs.heading,
        drive.getPositions(),
        value
      )
    }

  var pureVisionPose: Pose2d = Pose2d()

  init {
    SmartDashboard.putData("Elastic Swerve Drive") { builder: SendableBuilder ->
      builder.setSmartDashboardType("SwerveDrive")
      builder.addDoubleProperty("Front Left Angle", { drive.modules[0].state.angle.radians }, null)
      builder.addDoubleProperty("Front Left Velocity", { drive.modules[0].state.speedMetersPerSecond }, null)

      builder.addDoubleProperty("Front Right Angle", { drive.modules[1].state.angle.radians }, null)
      builder.addDoubleProperty("Front Right Velocity", { drive.modules[1].state.speedMetersPerSecond }, null)

      builder.addDoubleProperty("Back Left Angle", { drive.modules[2].state.angle.radians }, null)
      builder.addDoubleProperty("Back Left Velocity", { drive.modules[2].state.speedMetersPerSecond }, null)

      builder.addDoubleProperty("Back Right Angle", { drive.modules[3].state.angle.radians }, null)
      builder.addDoubleProperty("Back Right Velocity", { drive.modules[3].state.speedMetersPerSecond }, null)

      builder.addDoubleProperty("Robot Angle", { poseEstimator.estimatedPosition.rotation.radians }, null)
    }
  }

  fun resetOdometry(newPose: Pose2d) {
    this.poseEstimator.resetPose(newPose)
  }

  /**
   * pathfinds to a pose while allowing driver control through magnetization
   * @param desState pose to travel to
   */
  fun setMagnetizePathplanning(desState: Pose2d) {

    val xController = PIDController(AutoConstants.DEFAULT_X_KP, 0.0, 0.0)
    val yController = PIDController(AutoConstants.DEFAULT_Y_KP, 0.0, 0.0)
    val thetaController = PIDController(AutoConstants.DEFAULT_ROTATION_KP, 0.0, 0.0)

    val xPID = xController.calculate(pose.x, desState.x)
    val yPID = yController.calculate(pose.y, desState.y)
    val angPID = thetaController.calculate(pose.rotation.radians, desState.rotation.radians)

    ChassisSpeeds.fromFieldRelativeSpeeds(xPID, yPID, angPID, desState.rotation)
  }

  override fun periodic() {
    oldPose = pose
    if (isReal) {
      this.poseEstimator.update(
        ahrs.heading,
        drive.getPositions()
      )
    } else {
      drive as SwerveSim
      this.poseEstimator.update(
        drive.currHeading,
        drive.getPositions()
      )
    }

    if (cameras.isNotEmpty()) localize()

    setRobotPose()
  }

  private fun localize() = try {
    for ((index, camera) in cameras.withIndex()) {
      val results = camera.estimatedPose()
      for (result in results) {
        if (result.isPresent) {
          val presentResult = result.get()
          numTargets[index] = presentResult.targetsUsed.size.toDouble()
          tagDistance[index] = 0.0
          avgAmbiguity[index] = 0.0
          heightError[index] = abs(presentResult.estimatedPose.z)

          for (tag in presentResult.targetsUsed) {
            val tagPose = camera.estimator.fieldTags.getTagPose(tag.fiducialId)
            if (tagPose.isPresent) {
              val estimatedToTag = presentResult.estimatedPose.minus(tagPose.get())
              tagDistance[index] += sqrt(estimatedToTag.x.pow(2) + estimatedToTag.y.pow(2)) / numTargets[index]
              avgAmbiguity[index] = tag.poseAmbiguity / numTargets[index]
            } else {
              tagDistance[index] = Double.MAX_VALUE
              avgAmbiguity[index] = Double.MAX_VALUE
              break
            }
          }

          val estVisionPose = presentResult.estimatedPose.toPose2d()

          visionPose[0 + 3 * index] = estVisionPose.x
          visionPose[1 + 3 * index] = estVisionPose.y
          visionPose[2 + 3 * index] = estVisionPose.rotation.radians

          val inAmbiguityTolerance = avgAmbiguity[index] <= VisionConstants.MAX_AMBIGUITY
          val inDistanceTolerance = (numTargets[index] < 2 && tagDistance[index] <= VisionConstants.MAX_DISTANCE_SINGLE_TAG) ||
            (numTargets[index] >= 2 && tagDistance[index] <= VisionConstants.MAX_DISTANCE_MULTI_TAG + (numTargets[index] - 2) * VisionConstants.NUM_TAG_FACTOR)
          val inHeightTolerance = heightError[index] < VisionConstants.MAX_HEIGHT_ERR_METERS

          if (presentResult.timestampSeconds > 0 &&
            inGyroTolerance(estVisionPose.rotation) &&
            inAmbiguityTolerance &&
            inDistanceTolerance &&
            inHeightTolerance
          ) {
            if (enableVisionFusion) {
              val interpolatedPose = InterpolatedVision.interpolatePose(estVisionPose, index)

              poseEstimator.addVisionMeasurement(
                interpolatedPose,
                presentResult.timestampSeconds,
                camera.getEstimationStdDevs(numTargets[index].toInt(), tagDistance[index])
              )
            }
            usedVision[index] = true
            usedVisionSights[index] += 1.toLong()
          } else {
            usedVision[index] = false
            rejectedVisionSights[index] += 1.toLong()
          }
        }
      }
    }
  } catch (e: Error) {
    DriverStation.reportError(
      "!!!!!!!!! VISION ERROR !!!!!!!",
      e.stackTrace
    )
  }

  private fun inGyroTolerance(visionPoseRot: Rotation2d): Boolean {
    val currHeadingRad = if (isReal) {
      ahrs.heading.radians
    } else {
      drive as SwerveSim
      drive.currHeading.radians
    }

    val result = abs(
      MathUtil.angleModulus(
        MathUtil.angleModulus(visionPoseRot.radians) - MathUtil.angleModulus(currHeadingRad)
      )
    ) < VisionConstants.TAG_HEADING_MAX_DEV_RAD || abs(
      MathUtil.angleModulus(
        MathUtil.angleModulus(visionPoseRot.radians) - MathUtil.angleModulus(currHeadingRad)
      ) + 2 * PI
    ) < VisionConstants.TAG_HEADING_MAX_DEV_RAD || abs(
      MathUtil.angleModulus(
        MathUtil.angleModulus(visionPoseRot.radians) - MathUtil.angleModulus(currHeadingRad)
      ) - 2 * PI
    ) < VisionConstants.TAG_HEADING_MAX_DEV_RAD

    return result
  }

  fun getPosea(): Pose2d {
    return pose
  }

  private fun setRobotPose() {
    this.field.robotPose = this.pose

    this.field.getObject("FL").pose = this.pose.plus(
      Transform2d(
        drive.modules[0].location,
        drive.getPositions()[0].angle
      )
    )

    this.field.getObject("FR").pose = this.pose.plus(
      Transform2d(
        drive.modules[1].location,
        drive.getPositions()[1].angle
      )
    )

    this.field.getObject("BL").pose = this.pose.plus(
      Transform2d(
        drive.modules[2].location,
        drive.getPositions()[2].angle
      )
    )

    this.field.getObject("BR").pose = this.pose.plus(
      Transform2d(
        drive.modules[3].location,
        drive.getPositions()[0].angle
      )
    )
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Pose")
    builder.addDoubleArrayProperty("1.1 Estimated Pose", { doubleArrayOf(pose.x, pose.y, pose.rotation.radians) }, null)

    builder.publishConstString("2.0", "Vision Stats")
    builder.addBooleanArrayProperty("2.1 Used Last Vision Estimate?", { usedVision }, null)
    builder.addDoubleArrayProperty("2.2 Number of Targets", { numTargets }, null)
    builder.addDoubleArrayProperty("2.3 Avg Tag Distance", { tagDistance }, null)
    builder.addDoubleArrayProperty("2.4 Average Ambiguity", { avgAmbiguity }, null)
    builder.addDoubleArrayProperty("2.5 Cam Height Error", { heightError }, null)
    builder.addIntegerArrayProperty("2.6 Total Used Vision Sights", { usedVisionSights }, null)
    builder.addIntegerArrayProperty("2.7 Total Rejected Vision Sights", { rejectedVisionSights }, null)
    for ((index, _) in cameras.withIndex()) {
      builder.addDoubleArrayProperty("2.8${1 + index} Vision Pose Cam $index", { visionPose.slice(IntRange(0 + 3 * index, 2 + 3 * index)).toDoubleArray() }, null)
    }
    builder.addBooleanProperty("2.9 Enabled Vision Fusion", { enableVisionFusion }, null)

    builder.publishConstString("3.0", "AHRS Values")
    builder.addDoubleProperty("3.1 Heading Degrees", { ahrs.heading.degrees }, null)
    builder.addDoubleProperty("3.2 Pitch Degrees", { ahrs.pitch.degrees }, null)
    builder.addDoubleProperty("3.3 Roll Degrees", { ahrs.roll.degrees }, null)
    builder.addDoubleProperty("3.4 Angular X Vel", { ahrs.angularXVel() }, null)
    builder.addBooleanProperty("3.5 Navx Connected", { ahrs.connected() }, null)
    builder.addBooleanProperty("3.6 Navx Calibrated", { ahrs.calibrated() }, null)
  }

  companion object {
    fun createPoseSubsystem(ahrs: AHRS, drive: SwerveDrive, field: Field2d, controller: CommandXboxController): PoseSubsystem {
      return PoseSubsystem(
        ahrs,
        VisionConstants.ESTIMATORS,
        drive,
        field,
        controller
      )
    }
  }
}
