package frc.team449.subsystems.drive

import choreo.trajectory.SwerveSample
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Subsystem

/** A drivetrain that uses closed-loop velocity control. */
interface DriveSubsystem : Subsystem {

  private val xController: PIDController
    get() = PIDController(10.0, 0.0, 0.0)
  private val yController: PIDController
    get() = PIDController(10.0, 0.0, 0.0)
  private val headingController: PIDController
    get() = PIDController(7.5, 0.0, 0.0)

  var heading: Rotation2d
    get() = Rotation2d(MathUtil.angleModulus(this.pose.rotation.radians))
    set(value) {
      this.pose = Pose2d(Translation2d(this.pose.x, this.pose.y), value)
    }

  var pose: Pose2d

  fun getPos(): Pose2d {
    return Pose2d(pose.x, pose.y, heading)
  }
  fun followTrajectory(sample: SwerveSample) {
    // Get the current pose of the robot
    val pose: Pose2d = getPos()

    // Generate the next speeds for the robot
    val speeds: ChassisSpeeds = ChassisSpeeds(
      sample.vx + xController.calculate(pose.x, sample.x),
      sample.vy + yController.calculate(pose.y, sample.y),
      sample.omega + headingController.calculate(pose.rotation.radians, sample.heading)
    )

    // Apply the generated speeds
    driveFieldRelative(speeds)
  }

  fun driveFieldRelative(speeds: ChassisSpeeds)

  /** Sets the drivetrain's desired speeds. */
  fun set(desiredSpeeds: ChassisSpeeds){

  }

  /** Sets all the robot's drive motors to 0. */
  fun stop(){

  }

  /**
   * Used to simulate a drivetrain. Only one instance of this class should be made per drivetrain.
   */
  interface SimController {
    fun update()

    /**
     * Simulate the current drawn by the drivetrain
     */
    fun getCurrentDraw(): Double
  }
}
