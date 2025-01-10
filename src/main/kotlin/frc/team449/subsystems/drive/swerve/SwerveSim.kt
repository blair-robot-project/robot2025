package frc.team449.subsystems.drive.swerve

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.wpilibj.Timer.getFPGATimestamp
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import kotlin.math.hypot

class SwerveSim(
  modules: List<SwerveModule>,
  maxLinearSpeed: Double,
  maxRotSpeed: Double,
  field: Field2d,
  maxModuleSpeed: Double
) : SwerveDrive(modules, maxLinearSpeed, maxRotSpeed, field, maxModuleSpeed) {

  private var lastTime = getFPGATimestamp()
  var currHeading = Rotation2d()

  private val odometryTracker = SwerveDriveOdometry(
    kinematics,
    currHeading,
    getPositions(),
    Pose2d()
  )

  var odometryPose: Pose2d = odometryTracker.poseMeters

  override fun periodic() {
    val currTime = getFPGATimestamp()

    currHeading = currHeading.plus(Rotation2d(super.desiredSpeeds.omegaRadiansPerSecond * (currTime - lastTime)))
    this.lastTime = currTime

    driveFieldRelative(super.desiredSpeeds)

    // Updates the robot's currentSpeeds.
    currentSpeeds = kinematics.toChassisSpeeds(
      arrayOf(
        modules[0].state,
        modules[1].state,
        modules[2].state,
        modules[3].state
      )
    )

    odometryPose = odometryTracker.update(
      currHeading,
      getPositions()
    )

    speedMagnitude = hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
  }

  fun resetOdometryOnly(pose: Pose2d) {
    odometryTracker.resetPosition(
      currHeading,
      getPositions(),
      pose
    )
  }
}
