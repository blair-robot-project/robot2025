package frc.team449

import edu.wpi.first.networktables.BooleanSubscriber
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.drive.swerve.SwerveOrthogonalCommand
import frc.team449.subsystems.light.Light.Companion.createLight
import frc.team449.subsystems.vision.PoseSubsystem.Companion.createPoseSubsystem
import frc.team449.system.AHRS
import monologue.Annotations.Log
import monologue.Logged


class Robot : RobotBase(), Logged {

  val driveController = CommandXboxController(0)

  val mechController = CommandXboxController(1)

  val ahrs = AHRS()

  // Instantiate/declare PDP and other stuff here
  @Log.NT
  override val powerDistribution: PowerDistribution = PowerDistribution(
    RobotConstants.PDH_CAN,
    PowerDistribution.ModuleType.kRev
  )

  @Log.NT
  override val drive = SwerveDrive.createSwerveKraken(field)

  @Log.NT
  override val poseSubsystem = createPoseSubsystem(ahrs, drive, field)

  @Log.NT
  override val driveCommand = SwerveOrthogonalCommand(drive, poseSubsystem, driveController.hid)

  val light = createLight()

  /** Example for using infrared sensors
   * @Log.NT
   * val infrared = DigitalInput(RobotConstants.IR_CHANNEL)
   */
}
