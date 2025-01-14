package frc.team449

import edu.wpi.first.epilogue.Logged
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.drive.swerve.SwerveOrthogonalCommand
import frc.team449.subsystems.elevator.Elevator
import frc.team449.subsystems.elevator.Elevator.Companion.createElevator
import frc.team449.subsystems.light.Light.Companion.createLight
import frc.team449.subsystems.pivot.Pivot
import frc.team449.subsystems.pivot.Pivot.Companion.createPivot
import frc.team449.subsystems.vision.PoseSubsystem
import frc.team449.subsystems.vision.PoseSubsystem.Companion.createPoseSubsystem
import frc.team449.system.AHRS

@Logged
class Robot : RobotBase() {

    val driveController = CommandXboxController(0)

    val mechController = CommandXboxController(1)

    val ahrs = AHRS()

    // Instantiate/declare PDP and other stuff here
    override val powerDistribution: PowerDistribution =
        PowerDistribution(RobotConstants.PDH_CAN, PowerDistribution.ModuleType.kRev)

    override val drive: SwerveDrive = SwerveDrive.createSwerveKraken(field)

    override val poseSubsystem: PoseSubsystem = createPoseSubsystem(ahrs, drive, field)

    override val driveCommand: SwerveOrthogonalCommand =
        SwerveOrthogonalCommand(drive, poseSubsystem, driveController.hid)

    val elevator: Elevator = createElevator()

    val pivot: Pivot = createPivot()

    val light = createLight()
}
