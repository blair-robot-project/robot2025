package frc.team449.subsystems.vision

import dev.doglog.DogLog
import edu.wpi.first.epilogue.Logged
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.control.vision.ApriltagCamera
import frc.team449.subsystems.FieldConstants
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.drive.swerve.SwerveSim
import frc.team449.system.AHRS
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt

@Logged
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

  fun resetOdometry(pose: Pose2d) {
    poseEstimator.resetPose(pose)
  }

  fun isPivotSide(): Boolean {
    val closestReefRadians = pose.nearest(FieldConstants.REEF_CENTER_LOCATIONS).rotation.radians

    return !(
      abs(MathUtil.angleModulus(closestReefRadians - heading.radians)) <
        abs(MathUtil.angleModulus(MathUtil.angleModulus(PI + closestReefRadians) - heading.radians))
      )
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

    logData()
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
//              val interpolatedPose = InterpolatedVision.interpolatePose(estVisionPose, index)

              poseEstimator.addVisionMeasurement(
                estVisionPose,
                presentResult.timestampSeconds,
                camera.getEstimationStdDevs(numTargets[index].toInt(), tagDistance[index])
              )
              usedVision[index] = true
              usedVisionSights[index] += 1.toLong()
            }
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

  private fun logData() {
    DogLog.log("PoseSubsystem/Estimated Pose", pose)

    DogLog.log("PoseSubsystem/Vision Stats/Used Last Vision Estimate", usedVision)
    DogLog.log("PoseSubsystem/Vision Stats/Number of Targets", numTargets)
    DogLog.log("PoseSubsystem/Vision Stats/Avg Tag Distance", tagDistance)
    DogLog.log("PoseSubsystem/Vision Stats/Average Ambiguity", avgAmbiguity)
    DogLog.log("PoseSubsystem/Vision Stats/Cam Height Error", heightError)
    DogLog.log("PoseSubsystem/Vision Stats/Total Used Vision Sights", usedVisionSights)
    DogLog.log("PoseSubsystem/Vision Stats/Total Rejected Vision Sights", rejectedVisionSights)
    for ((index, _) in cameras.withIndex()) {
      DogLog.log("PoseSubsystem/Vision Stats/Vision Pose Cam $index", visionPose.slice(IntRange(0 + 3 * index, 2 + 3 * index)).toDoubleArray())
    }
    DogLog.log("PoseSubsystem/Vision Stats/Enabled Vision Fusion", enableVisionFusion)

    DogLog.log("PoseSubsystem/AHRS Values/Heading Degrees", ahrs.heading.degrees)
    DogLog.log("PoseSubsystem/AHRS Values/Pitch Degrees", ahrs.pitch.degrees)
    DogLog.log("PoseSubsystem/AHRS Values/Roll Degrees", ahrs.roll.degrees)
    DogLog.log("PoseSubsystem/AHRS Values/Angular X Vel", ahrs.angularXVel())
    DogLog.log("PoseSubsystem/AHRS Values/Navx Connected", ahrs.connected())
    DogLog.log("PoseSubsystem/AHRS Values/Navx Calibrated", ahrs.calibrated())
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
