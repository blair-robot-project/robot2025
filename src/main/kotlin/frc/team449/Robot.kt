package frc.team449

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.commands.autoscoreCommands.pathfinder
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.drive.swerve.SwerveOrthogonalCommand
import frc.team449.subsystems.elevator.Elevator
import frc.team449.subsystems.elevator.Elevator.Companion.createElevator
import frc.team449.subsystems.light.Light.Companion.createLight
import frc.team449.subsystems.pivot.Pivot
import frc.team449.subsystems.pivot.Pivot.Companion.createPivot
import frc.team449.subsystems.superstructure.SuperstructureManager
import frc.team449.subsystems.superstructure.SuperstructureManager.Companion.createSuperstructureManager
import frc.team449.subsystems.vision.PoseSubsystem
import frc.team449.subsystems.vision.PoseSubsystem.Companion.createPoseSubsystem
import frc.team449.subsystems.wrist.Wrist
import frc.team449.subsystems.wrist.Wrist.Companion.createWrist
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
  override val drive: SwerveDrive = SwerveDrive.createSwerveKraken(field, this, driveController)

  @Log.NT
  override val poseSubsystem: PoseSubsystem = createPoseSubsystem(ahrs, drive, field, driveController)

  @Log.NT
  override val driveCommand: SwerveOrthogonalCommand = SwerveOrthogonalCommand(drive, poseSubsystem, driveController.hid)

  @Log.NT
  val elevator: Elevator = createElevator()

  @Log.NT
  val pivot: Pivot = createPivot()

  @Log.NT
  val wrist: Wrist = createWrist()

  val superstructureManager: SuperstructureManager = createSuperstructureManager(this)

  val light = createLight()

  val pathfinder = pathfinder(this)
}
