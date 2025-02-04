package frc.team449

import choreo.auto.AutoChooser
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.drive.swerve.SwerveOrthogonalCommand
import frc.team449.subsystems.light.Light.Companion.createLight
import frc.team449.subsystems.vision.PoseSubsystem
import frc.team449.subsystems.vision.PoseSubsystem.Companion.createPoseSubsystem
import frc.team449.system.AHRS
import monologue.Annotations.Log
import monologue.Logged

class Robot : RobotBase(), Logged {

  // Driver/Operator Controllers
  val driveController: CommandXboxController = CommandXboxController(0)
  val mechController: CommandXboxController = CommandXboxController(1)

  // NavX
  val ahrs: AHRS = AHRS()

  // Instantiate/declare PDP and other stuff here
  @Log.NT
  override val powerDistribution: PowerDistribution = PowerDistribution(
    RobotConstants.PDH_CAN,
    PowerDistribution.ModuleType.kRev
  )

  @Log.NT
  override val drive: SwerveDrive = SwerveDrive.createSwerveNEO(field)

  val autoChooser = AutoChooser()


  @Log.NT
  override val poseSubsystem: PoseSubsystem = createPoseSubsystem(ahrs, drive, field)

  @Log.NT
  override val driveCommand: SwerveOrthogonalCommand = SwerveOrthogonalCommand(drive, poseSubsystem, driveController.hid)

  val light = createLight()
}
