package frc.team449

import com.ctre.phoenix6.SignalLogger
import dev.doglog.DogLog
import dev.doglog.DogLogOptions
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import frc.team449.auto.Routines
import frc.team449.subsystems.FieldConstants
import frc.team449.subsystems.drive.swerve.SwerveSim
import frc.team449.subsystems.superstructure.SuperstructureGoal
import frc.team449.subsystems.superstructure.elevator.ElevatorConstants
import frc.team449.subsystems.superstructure.elevator.ElevatorFeedForward.Companion.createElevatorFeedForward
import frc.team449.subsystems.superstructure.pivot.PivotFeedForward.Companion.createPivotFeedForward
import frc.team449.subsystems.superstructure.wrist.WristFeedForward.Companion.createWristFeedForward
import frc.team449.subsystems.vision.VisionConstants
import frc.team449.system.encoder.QuadCalibration
import org.littletonrobotics.urcl.URCL
import kotlin.jvm.optionals.getOrDefault
import kotlin.math.*

/** The main class of the robot, constructs all the subsystems
 * and initializes default commands . */
class RobotLoop : TimedRobot() {

  private val robot = Robot()
  val routines = Routines(robot)
  private val field = robot.field

  private var componentStorage: Array<Pose3d> = arrayOf(
    Pose3d(),
    Pose3d(),
    Pose3d(),
    Pose3d(),
    Pose3d(
      0.0,
      0.0,
      0.0,
      Rotation3d(0.0, 0.0, 0.0)
    )
  )

  private val controllerBinder = ControllerBindings(robot.driveController, robot.mechController, robot.characController, robot)

  override fun robotInit() {
    // Yes this should be a print statement, it's useful to know that robotInit started.
    println("Started robotInit.")

    SignalLogger.setPath("/media/sda1/ctre-logs/")
    SignalLogger.start()

    HAL.report(FRCNetComm.tResourceType.kResourceType_Language, FRCNetComm.tInstances.kLanguage_Kotlin)

//    if (isSimulation()) {
    // Don't complain about joysticks if there aren't going to be any
    DriverStation.silenceJoystickConnectionWarning(true)
//      val instance = NetworkTableInstance.getDefault()
//      instance.stopServer()
//      instance.startClient4("localhost")
//    }

    // Custom Feedforwards
    robot.elevator.elevatorFeedForward = createElevatorFeedForward(robot.pivot)
    robot.pivot.pivotFeedForward = createPivotFeedForward(robot.elevator)
    robot.wrist.wristFeedForward = createWristFeedForward(robot.pivot)

    // Generate Auto Routines
    println("Generating Auto Routines : ${Timer.getFPGATimestamp()}")

    routines.addOptions(robot.autoChooser)

    SmartDashboard.putData("Auto Chooser", robot.autoChooser)

    RobotModeTriggers.autonomous().whileTrue(robot.autoChooser.selectedCommandScheduler())
    println("DONE Generating Auto Routines : ${Timer.getFPGATimestamp()}")
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance())

    controllerBinder.bindButtons()

    DogLog.setOptions(
      DogLogOptions()
        .withCaptureDs(true)
        .withCaptureNt(true)
    )

    SmartDashboard.putData("Field", robot.field)
    SmartDashboard.putData("Elevator + Pivot Visual", robot.elevator.mech)

    URCL.start()

    QuadCalibration(robot.pivot, robot.pivot.absoluteEncoder, robot.pivot.quadEncoder, name = "Pivot")
      .ignoringDisable(true)
      .schedule()

//    QuadCalibration(robot.wrist, robot.wrist.absoluteEncoder, robot.wrist.quadEncoder, name = "Wrist")
//      .ignoringDisable(true)
//      .schedule()
    if (RobotBase.isReal()) {
      robot.wrist.startupZero()
    }
  }

  override fun driverStationConnected() {
    FieldConstants.configureReef(DriverStation.getAlliance().getOrDefault(DriverStation.Alliance.Blue))
  }

  override fun robotPeriodic() {
    CommandScheduler.getInstance().run()

    // Robot Drive Logging
    robot.field.robotPose = robot.poseSubsystem.pose
    robot.field.getObject("bumpers").pose = robot.poseSubsystem.pose

    logAdvScopeComponents()
  }

  override fun autonomousInit() {
    /** Every time auto starts, we update the chosen auto command. */
  }

  override fun autonomousPeriodic() {}

  override fun teleopInit() {
    robot.superstructureManager.requestGoal(SuperstructureGoal.STOW).schedule()

    (robot.light.currentCommand ?: InstantCommand()).cancel()

    robot.drive.defaultCommand = robot.driveCommand
  }

  override fun teleopPeriodic() {
  }

  override fun disabledInit() {
    robot.drive.stop()
  }

  override fun disabledPeriodic() {}

  override fun testInit() {}

  override fun testPeriodic() {}

  override fun simulationInit() {}

  override fun simulationPeriodic() {
    // Superstructure Simulation
    robot.elevator.elevatorLigament.length = ElevatorConstants.MIN_VIS_HEIGHT + robot.elevator.positionSupplier.get()
    robot.elevator.desiredElevatorLigament.length = ElevatorConstants.MIN_VIS_HEIGHT + robot.elevator.targetSupplier.get()

    robot.elevator.elevatorLigament.angle = Units.radiansToDegrees(robot.pivot.positionSupplier.get())
    robot.elevator.desiredElevatorLigament.angle = Units.radiansToDegrees(robot.pivot.targetSupplier.get())

    robot.elevator.wristLigament.angle = Units.radiansToDegrees(robot.wrist.positionSupplier.get())

    robot.drive as SwerveSim

    VisionConstants.ESTIMATORS.forEach {
      it.simulationPeriodic(robot.drive.odometryPose)
    }

    VisionConstants.VISION_SIM.debugField.getObject("EstimatedRobot").pose = robot.poseSubsystem.pose

    // change elevator angle according to pivot position
    robot.elevator.elevatorSim?.changeAngle(robot.pivot.positionSupplier.get())
  }

  private fun logAdvScopeComponents() {
    val pivotPos = -robot.pivot.positionSupplier.get()
    val cosPivot = cos(-pivotPos)
    val sinPivot = sin(-pivotPos)

    val elevatorPos = robot.elevator.positionSupplier.get()

    componentStorage = arrayOf(
//       pivot/base stage
      Pose3d(
        -0.136,
        0.0,
        0.245,
        Rotation3d(0.0, pivotPos, 0.0)
      ),
      // first stage max: 0.60
      Pose3d(
        -0.136 + min(0.6 * cosPivot, elevatorPos * cosPivot),
        0.0,
        0.245 + min(0.6 * sinPivot, elevatorPos * sinPivot),
        Rotation3d(0.0, pivotPos, 0.0)
      ),
      // second stage max : 0.575 (1.175)
      Pose3d(
        -0.136 + min(1.175 * cosPivot, elevatorPos * cosPivot),
        0.0,
        0.245 + min(1.175 * sinPivot, elevatorPos * sinPivot),
        Rotation3d(0.0, pivotPos, 0.0)
      ),
      // third stage max: 0.56 (1.735)
      Pose3d(
        -0.136 + min(1.735 * cosPivot, elevatorPos * cosPivot),
        0.0,
        0.245 + min(1.735 * sinPivot, elevatorPos * sinPivot),
        Rotation3d(0.0, pivotPos, 0.0)
      ),
      Pose3d(
        -.136 + (0.7112 * cosPivot + (0.127 * -sinPivot)) +
          min(1.735 * cosPivot, elevatorPos * cosPivot),
        0.0,
        .245 + (0.7112 * sinPivot) + (0.127 * cosPivot) +
          min(1.735 * sinPivot, elevatorPos * sinPivot),
        Rotation3d(0.0, -robot.wrist.positionSupplier.get() + pivotPos, 0.0)
      )
    )
    DogLog.log("AdvScopeComponents", componentStorage)
  }
}
