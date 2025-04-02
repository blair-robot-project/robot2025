package frc.team449

import choreo.auto.AutoChooser
import com.ctre.phoenix6.configs.TalonFXConfiguration
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.drive.swerve.SwerveOrthogonalCommand
import frc.team449.subsystems.light.Light.Companion.createLight
import frc.team449.subsystems.superstructure.SuperstructureManager
import frc.team449.subsystems.superstructure.SuperstructureManager.Companion.createSuperstructureManager
import frc.team449.subsystems.superstructure.climb.Climb
import frc.team449.subsystems.superstructure.climb.Climb.Companion.createClimb
import frc.team449.subsystems.superstructure.elevator.Elevator
import frc.team449.subsystems.superstructure.elevator.Elevator.Companion.createElevator
import frc.team449.subsystems.superstructure.intake.Intake
import frc.team449.subsystems.superstructure.intake.Intake.Companion.createIntake
import frc.team449.subsystems.superstructure.pivot.Pivot
import frc.team449.subsystems.superstructure.pivot.Pivot.Companion.createPivot
import frc.team449.subsystems.superstructure.wrist.Wrist
import frc.team449.subsystems.superstructure.wrist.Wrist.Companion.createWrist
import frc.team449.subsystems.vision.PoseSubsystem
import frc.team449.subsystems.vision.PoseSubsystem.Companion.createPoseSubsystem
import frc.team449.subsystems.voltage.VoltageDistribution
import frc.team449.system.AHRS

class Robot : RobotBase() {

  // Driver/Operator Controllers
  val driveController: CommandXboxController = CommandXboxController(0)
  val mechController: CommandXboxController = CommandXboxController(1)
  val characController: CommandXboxController = CommandXboxController(2)

  // NavX
  val ahrs: AHRS = AHRS()

  // Instantiate/declare PDP and other stuff here
  override val powerDistribution: PowerDistribution = PowerDistribution(
    RobotConstants.PDH_CAN,
    PowerDistribution.ModuleType.kRev,
  )

  fun setSupplyLim(config: TalonFXConfiguration, limit: Double) {
    config.CurrentLimits.StatorCurrentLimit = limit
  }

  val voltageDistribution: VoltageDistribution = VoltageDistribution(powerDistribution)

  override val drive: SwerveDrive = SwerveDrive.createSwerveKraken(field, this)

  val autoChooser = AutoChooser()

  override val poseSubsystem: PoseSubsystem = createPoseSubsystem(ahrs, drive, field)

  override val driveCommand: SwerveOrthogonalCommand = SwerveOrthogonalCommand(drive, poseSubsystem, driveController.hid)

  val elevator: Elevator = createElevator(this)

  val pivot: Pivot = createPivot(this)

  val wrist: Wrist = createWrist(this)

  val intake: Intake = createIntake()

  val climb: Climb = createClimb()

  val superstructureManager: SuperstructureManager = createSuperstructureManager(this)

  val light = createLight()
}
