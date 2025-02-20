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
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
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
import frc.team449.commands.autoscoreCommands.AutoScoreCommandConstants


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


  //edem mag vars
  private var currentControllerPower = 15.0
  private var magMultiply = 1.00
  private val magIncConstant = 0.001
  private var magDec = 0.0004
  private val maxMagPower = 20.0
  private var lastDistance = 0.0
  private val agreeVal = 0.75
  val autoDistance = 1
  lateinit var autoscoreCurrentCommand : Command

  init {
    xController.reset()
    xController.setTolerance(poseTol.x)


    yController.reset()
    yController.setTolerance(poseTol.y)


    thetaController.reset()
    thetaController.enableContinuousInput(-PI, PI)
    thetaController.setTolerance(poseTol.rotation.radians)


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


    xController.reset()
    yController.reset()
    thetaController.reset()


    timer.restart()


    resetMagVars()
  }

  

  private fun resetMagVars() {
    currentControllerPower = 15.0
    magMultiply = 1.00
    magDec = 0.0004
    lastDistance = 0.0
  }


  private fun clampCP() {
    currentControllerPower = MathUtil.clamp(currentControllerPower, 0.1, maxMagPower)
  }


  private fun clampMult() {
    magMultiply = MathUtil.clamp(magMultiply, 0.1, 2.0)
  }


  fun pathfindingMagnetize(desVel: ChassisSpeeds) {
    val currTime = timer.get()
    dt = currTime - prevTime
    prevTime = currTime

    val distance = pose.translation.getDistance(autoscoreCommandPose.translation)
    val ctrlX = -controller.leftY
    val ctrlY = -controller.leftX
    val controllerMag = hypot(ctrlX, ctrlY)

    if(distance > lastDistance) {
      magMultiply += magIncConstant
      magDec -= 0.001
    } else if(distance < lastDistance) {
      magMultiply -= magDec
      magDec += 0.001
    }
    magDec = MathUtil.clamp(magDec, 0.0, 0.5)
    clampMult()

    if(controllerMag > 0.35) {
      currentControllerPower = MathUtil.clamp(currentControllerPower, 15.0, maxMagPower)
      magMultiply += 0.3
    }

    //this increases the users power if they are moving a lot
    //also decreases the users power if they are not moving a lot
    currentControllerPower += 1.2/( 1 + exp(-6 * (controllerMag-0.7) ) ) - 0.5

    currentControllerPower *= magMultiply

    clampCP()
    if(distance <= autoDistance || controllerMag < 0.1 || currentControllerPower < 3) {
      if(distance <= autoDistance) {
        resetMagVars()
      } else {
        currentControllerPower -= 0.1
        magMultiply -= 0.05
      }
      println("set pathmag without controller")
      drive.set(desVel)
    } else {
      // controller stuff

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
          rotCtrl.calculate(heading.radians),
          -RobotConstants.ALIGN_ROT_SPEED,
          RobotConstants.ALIGN_ROT_SPEED
        )
      }


      val vel = Translation2d(xClamped, yClamped)


      vel.rotateBy(Rotation2d(-rotScaled * dt * skewConstant))


      var controllerSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        vel.x * directionCompensation.invoke(),
        vel.y * directionCompensation.invoke(),
        rotScaled,
        heading
      )
      //this increases the users power based on how much it is going against pathmag
      if(controllerSpeeds.vxMetersPerSecond < 0 != desVel.vxMetersPerSecond < 0) {
        currentControllerPower += (abs(controllerSpeeds.vxMetersPerSecond))/20
      } else if(controllerSpeeds.vyMetersPerSecond < 0 != desVel.vyMetersPerSecond < 0) {
        currentControllerPower += (abs(controllerSpeeds.vyMetersPerSecond))/20
      }



      controllerSpeeds *= currentControllerPower / 2
      val desVelAdjustedSpeeds = desVel / (20 / ( 1 + exp(-(currentControllerPower-12)/1) ) )

//      println(" x ${
//        abs(MathUtil.clamp(controllerSpeeds.vxMetersPerSecond, -1.0, 1.0) - MathUtil.clamp(desVel.vxMetersPerSecond, -1.0, 1.0))
//      } y ${abs(MathUtil.clamp(controllerSpeeds.vyMetersPerSecond, -1.0, 1.0)  - MathUtil.clamp(desVel.vyMetersPerSecond, -1.0, 1.0))}")

      val combinedChassisSpeeds : ChassisSpeeds
      if( abs(MathUtil.clamp(controllerSpeeds.vxMetersPerSecond, -1.0, 1.0)  - MathUtil.clamp(desVel.vxMetersPerSecond, -1.0, 1.0) ) +
        abs(MathUtil.clamp(controllerSpeeds.vyMetersPerSecond, -1.0, 1.0)  - MathUtil.clamp(desVel.vyMetersPerSecond, -1.0, 1.0) )
        < agreeVal
        ) {
        combinedChassisSpeeds = desVel
        currentControllerPower = MathUtil.clamp(currentControllerPower, 0.1, 5.0)
        currentControllerPower -= 0.3
//        println("agree")
      } else if(controllerMag > 0.95 || currentControllerPower > 16) {
        combinedChassisSpeeds = controllerSpeeds
      } else {
        combinedChassisSpeeds = controllerSpeeds + desVelAdjustedSpeeds
        combinedChassisSpeeds.omegaRadiansPerSecond = controllerSpeeds.omegaRadiansPerSecond
      }

      combinedChassisSpeeds.vxMetersPerSecond = MathUtil.clamp(combinedChassisSpeeds.vxMetersPerSecond , -AutoScoreCommandConstants.MAX_LINEAR_SPEED, AutoScoreCommandConstants.MAX_LINEAR_SPEED)
      combinedChassisSpeeds.vyMetersPerSecond = MathUtil.clamp(combinedChassisSpeeds.vyMetersPerSecond , -AutoScoreCommandConstants.MAX_LINEAR_SPEED, AutoScoreCommandConstants.MAX_LINEAR_SPEED)
      combinedChassisSpeeds.omegaRadiansPerSecond = MathUtil.clamp(combinedChassisSpeeds.omegaRadiansPerSecond, -AutoScoreCommandConstants.MAX_ROT_SPEED, AutoScoreCommandConstants.MAX_ROT_SPEED)
      println("setting pathmag with controller")
      drive.set(combinedChassisSpeeds)

    }

//    println("controller mag ${controllerMag}")
    lastDistance = distance
//    println("controller power: $currentControllerPower mult: $magMultiply")

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


//  fun setMagnetizePathplanning(desState: Pose2d) {
//
//
//    val xController = PIDController(AutoConstants.DEFAULT_X_KP, 0.0, 0.0)
//    val yController = PIDController(AutoConstants.DEFAULT_Y_KP, 0.0, 0.0)
//    val thetaController = PIDController(AutoConstants.DEFAULT_ROTATION_KP, 0.0, 0.0)
//
//
//    val xPID = xController.calculate(pose.x, desState.x)
//    val yPID = yController.calculate(pose.y, desState.y)
//    val angPID = thetaController.calculate(pose.rotation.radians, desState.rotation.radians)
//
//
//    ChassisSpeeds.fromFieldRelativeSpeeds(xPID, yPID, angPID, desState.rotation)
//  }


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
      val results = camera.estimatedPose(pose)
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

