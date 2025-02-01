package frc.team449.subsystems.vision

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.control.vision.ApriltagCamera
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveConstants.TURN_KD
import frc.team449.subsystems.drive.swerve.SwerveConstants.TURN_KI
import frc.team449.subsystems.drive.swerve.SwerveConstants.TURN_KP
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.drive.swerve.SwerveSim
import frc.team449.subsystems.vision.interpolation.InterpolatedVision
import frc.team449.system.AHRS
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt

class PoseSubsystem(
  private val ahrs: AHRS,
  private val cameras: List<ApriltagCamera> = mutableListOf(),
  private val drive: SwerveDrive,
  private val field: Field2d
) : SubsystemBase() {

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

  override fun periodic() {
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

          if (presentResult.timestampSeconds > 0 &&
            inGyroTolerance(estVisionPose.rotation) &&
            avgAmbiguity[index] <= VisionConstants.MAX_AMBIGUITY &&
            (
              numTargets[index] < 2 && tagDistance[index] <= VisionConstants.MAX_DISTANCE_SINGLE_TAG ||
                numTargets[index] >= 2 && tagDistance[index] <= VisionConstants.MAX_DISTANCE_MULTI_TAG + (numTargets[index] - 2) * VisionConstants.NUM_TAG_FACTOR
              ) &&
            heightError[index] < VisionConstants.MAX_HEIGHT_ERR_METERS
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
  fun turnToDesiredDisplacement(desiredDisplacementDeg: Double): Command {
    val pidController = PIDController(TURN_KP, TURN_KI, TURN_KD).apply {
      setTolerance(Math.toRadians(2.0))
    }

    val desiredDisplacementRad = Math.toRadians(desiredDisplacementDeg)

    return run {

      run {
        val currentAngle = MathUtil.angleModulus(this.pose.rotation.radians)
        val omegaRadPerSec = pidController.calculate(currentAngle, desiredDisplacementRad)
        drive.set(ChassisSpeeds.fromRobotRelativeSpeeds(drive.desiredSpeeds.vxMetersPerSecond, drive.desiredSpeeds.vyMetersPerSecond, omegaRadPerSec, ahrs.heading))
        //need to make sure max rot speed isnt exceeded
      }
    }.until {
      pidController.atSetpoint()
    }.andThen(drive.stop())
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
    fun createPoseSubsystem(ahrs: AHRS, drive: SwerveDrive, field: Field2d): PoseSubsystem {
      return PoseSubsystem(
        ahrs,
        VisionConstants.ESTIMATORS,
        drive,
        field
      )
    }
  }
}
