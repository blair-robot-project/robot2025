package frc.team449.subsystems.drive.swerve

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.subsystems.vision.PoseSubsystem
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.hypot

class WheelRadiusCharacterization(
  val drive: SwerveDrive,
  val pose: PoseSubsystem
) : Command() {

  init {
    addRequirements(drive)
  }

  private var currentEffectiveWheelRadius = 0.0

  private var lastGyroYawRads = 0.0
  private var accumGyroYawRads = 0.0

  private var startWheelPositions = Array(drive.getPositions().size) { _ -> 0.0 }

  private var rateLimiter = SlewRateLimiter(PI / 4)

  private var requestedRotSpeed = Double.MAX_VALUE

  override fun initialize() {
    lastGyroYawRads = pose.heading.radians

    startWheelPositions = Array(drive.getPositions().size) { i -> 2 * PI * drive.getPositions()[i].distanceMeters / SwerveConstants.DRIVE_UPR }

    println("Starting wheel radius characterization")
  }

  override fun execute() {
    requestedRotSpeed = if (accumGyroYawRads < 3 * PI) {
      rateLimiter.calculate(PI / 2)
    } else {
      rateLimiter.calculate(0.0)
    }

    drive.set(
      ChassisSpeeds(
        0.0,
        0.0,
        requestedRotSpeed
      )
    )

    accumGyroYawRads += MathUtil.angleModulus(pose.heading.radians - lastGyroYawRads)
    lastGyroYawRads = pose.heading.radians

    var averageWheelPosition = 0.0
    val wheelPositions = Array(drive.getPositions().size) { i -> 2 * PI * drive.getPositions()[i].distanceMeters / SwerveConstants.DRIVE_UPR }

    for (i in 0..3) {
      averageWheelPosition += abs(wheelPositions[i] - startWheelPositions[i])
    }

    averageWheelPosition /= 4.0

    val driveRadius = hypot(SwerveConstants.WHEELBASE, SwerveConstants.TRACKWIDTH) / 2.0

    currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition
  }

  override fun isFinished(): Boolean {
    return requestedRotSpeed == 0.0
  }

  override fun end(interrupted: Boolean) {
    drive.stop()

    println("\nEffective Wheel Radius: ${Units.metersToInches(currentEffectiveWheelRadius)} inches.")
  }
}
