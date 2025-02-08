package frc.team449

import com.ctre.phoenix6.SignalLogger
import dev.doglog.DogLog
import dev.doglog.DogLogOptions
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.RunCommand
import frc.team449.auto.RoutineChooser
import frc.team449.commands.light.BlairChasing
import frc.team449.commands.light.BreatheHue
import frc.team449.commands.light.Rainbow
import frc.team449.subsystems.FieldConstants
import frc.team449.subsystems.drive.swerve.SwerveSim
import frc.team449.subsystems.superstructure.elevator.ElevatorConstants
import frc.team449.subsystems.superstructure.elevator.ElevatorFeedForward.Companion.createElevatorFeedForward
import frc.team449.subsystems.superstructure.pivot.PivotFeedForward.Companion.createPivotFeedForward
import frc.team449.subsystems.vision.VisionConstants
import frc.team449.system.encoder.QuadCalibration
import org.littletonrobotics.urcl.URCL
import kotlin.jvm.optionals.getOrDefault
import kotlin.jvm.optionals.getOrNull
import kotlin.math.*

/** The main class of the robot, constructs all the subsystems
 * and initializes default commands . */
class RobotLoop : TimedRobot() {

  private val robot = Robot()

  private val routineChooser: RoutineChooser = RoutineChooser(robot)

  private var autoCommand: Command? = null
  private var routineMap = hashMapOf<String, Command>()
  private val controllerBinder = ControllerBindings(robot.driveController, robot.mechController, robot)

  private var componentStorage: Array<Pose3d>? = null
  private var wristMove: Pose3d? = null
  var offset = 0.0


  override fun robotInit() {
    // Yes this should be a print statement, it's useful to know that robotInit started.
    println("Started robotInit.")

    SignalLogger.setPath("/media/sda1/ctre-logs/")
    SignalLogger.start()

    HAL.report(FRCNetComm.tResourceType.kResourceType_Language, FRCNetComm.tInstances.kLanguage_Kotlin)

    if (isSimulation()) {
      // Don't complain about joysticks if there aren't going to be any
      DriverStation.silenceJoystickConnectionWarning(true)
//      val instance = NetworkTableInstance.getDefault()
//      instance.stopServer()
//      instance.startClient4("localhost")
    }

    // Custom Feedforwards
    robot.elevator.elevatorFeedForward = createElevatorFeedForward(robot.pivot)
    robot.pivot.pivotFeedForward = createPivotFeedForward(robot.elevator)

    // Generate Auto Routines
    println("Generating Auto Routines : ${Timer.getFPGATimestamp()}")
    routineMap = routineChooser.routineMap()
    println("DONE Generating Auto Routines : ${Timer.getFPGATimestamp()}")

    routineChooser.createOptions()

    SmartDashboard.putData("Routine Chooser", routineChooser)
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance())

    robot.light.defaultCommand = BlairChasing(robot.light)

    controllerBinder.bindButtons()

    DogLog.setOptions(
      DogLogOptions()
        .withCaptureDs(true)
        .withCaptureNt(true)
        .withLogExtras(true)
    )

    DogLog.setPdh(robot.powerDistribution)

    SmartDashboard.putData("Field", robot.field)
    SmartDashboard.putData("Elevator + Pivot Visual", robot.elevator.mech)

    URCL.start()

    QuadCalibration(robot.pivot, robot.pivot.absoluteEncoder, robot.pivot.quadEncoder, name = "Pivot")
      .ignoringDisable(true)
      .schedule()

    QuadCalibration(robot.wrist, robot.wrist.absoluteEncoder, robot.wrist.quadEncoder, name = "Wrist")
      .ignoringDisable(true)
      .schedule()
  }

  override fun driverStationConnected() {
    FieldConstants.configureReef(DriverStation.getAlliance().getOrDefault(DriverStation.Alliance.Blue))
  }

  override fun robotPeriodic() {
    CommandScheduler.getInstance().run()

    // Robot Drive Logging
    robot.field.robotPose = robot.poseSubsystem.pose
    robot.field.getObject("bumpers").pose = robot.poseSubsystem.pose

    // Superstructure Simulation
    robot.elevator.elevatorLigament.length = ElevatorConstants.MIN_VIS_HEIGHT + robot.elevator.positionSupplier.get()
    robot.elevator.desiredElevatorLigament.length = ElevatorConstants.MIN_VIS_HEIGHT + robot.elevator.targetSupplier.get()

    robot.elevator.elevatorLigament.angle = Units.radiansToDegrees(robot.pivot.positionSupplier.get())
    robot.elevator.desiredElevatorLigament.angle = Units.radiansToDegrees(robot.pivot.targetSupplier.get())

    robot.elevator.wristLigament.angle = Units.radiansToDegrees(robot.wrist.positionSupplier.get())
  }

  override fun autonomousInit() {
    /** Every time auto starts, we update the chosen auto command. */
    this.autoCommand = routineMap[if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) "Red" + routineChooser.selected else "Blue" + routineChooser.selected]
    CommandScheduler.getInstance().schedule(this.autoCommand)

    if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) {
      BreatheHue(robot.light, 0).schedule()
    } else {
      BreatheHue(robot.light, 95).schedule()
    }
  }

  override fun autonomousPeriodic() {}

  override fun teleopInit() {
    if (autoCommand != null) {
      CommandScheduler.getInstance().cancel(autoCommand)
    }

    (robot.light.currentCommand ?: InstantCommand()).cancel()

    robot.drive.defaultCommand = robot.driveCommand
  }

  override fun teleopPeriodic() {
  }

  override fun disabledInit() {
    robot.drive.stop()

    (robot.light.currentCommand ?: InstantCommand()).cancel()
    Rainbow(robot.light).schedule()
  }

  override fun disabledPeriodic() {}

  override fun testInit() {
    if (autoCommand != null) {
      CommandScheduler.getInstance().cancel(autoCommand)
    }
  }

  override fun testPeriodic() {}

  override fun simulationInit() {
    // set position
//    componentStorage = arrayOf(
//      Pose3d(-0.136, 0.0, 0.245,
//        Rotation3d(0.0, 0.0, 0.0)),
//      Pose3d(-0.136, 0.0, 0.245,
//        Rotation3d(0.0, 0.0, 0.0)),
//      Pose3d(-0.136, 0.0, 0.245,
//        Rotation3d(0.0, 0.0, 0.0)),
//      Pose3d(-0.136, 0.0, 0.245,
//        Rotation3d(0.0, 0.0, 0.0)),
//      Pose3d(0.61, 0.0, 0.327,
//        Rotation3d(0.0, 0.0, 0.0)))
    componentStorage = arrayOf(Pose3d(),
      Pose3d(),
      Pose3d(),
      Pose3d(),
      Pose3d(0.0, 0.0, 0.0,
        Rotation3d(0.0, 0.0, 0.0)))
    robot.poseSubsystem.enableVisionFusion = false
//    robot.mechController.pov(90).whileTrue(RunCommand({offset += 0.01}).andThen(PrintCommand("yay")))
//    robot.mechController.pov(270).whileTrue(RunCommand({offset -= 0.01}).andThen(PrintCommand("naur")))
  }

  override fun simulationPeriodic() {
    // sim pose calculator

//    componentStorage[4]
    val pivotPos = -robot.pivot.positionSupplier.get()

    offset = robot.elevator.positionSupplier.get() / (ElevatorConstants.MAX_HEIGHT - ElevatorConstants.MIN_HEIGHT)
    offset *= 1.735

    offset = max(offset, 0.0)
    offset = min(offset, 1.735)

    componentStorage = arrayOf(
//       pivot/base stage
      Pose3d(-0.136, 0.0, 0.245,
        Rotation3d(0.0, pivotPos, 0.0)),
      // first stage max: 0.60
      Pose3d(-0.136 + min(0.6*cos(-pivotPos), offset*cos(-pivotPos)),
        0.0,
        0.245 + min(0.6*sin(-pivotPos),offset*sin(-pivotPos)),
        Rotation3d(0.0, pivotPos, 0.0)),
      // second stage max : 0.575 (1.175)
      Pose3d(-0.136 + min(1.175*cos(-pivotPos), offset*cos(-pivotPos)),
        0.0,
        0.245 + min(1.175*sin(-pivotPos),offset*sin(-pivotPos)),
        Rotation3d(0.0, pivotPos, 0.0)),
      // third stage max: 0.56 (1.735)
      Pose3d(-0.136 + min(1.735*cos(-pivotPos), offset*cos(-pivotPos)),
        0.0,
        0.245 + min(1.735*sin(-pivotPos),offset*sin(-pivotPos)),
        Rotation3d(0.0, pivotPos, 0.0)),
      Pose3d(-.136 + (0.7112*cos(-pivotPos)+(0.127*-sin(-pivotPos)))
        + min(1.735*cos(-pivotPos), offset*cos(-pivotPos)),
        0.0,
        .245 + (0.7112*sin(-pivotPos))+(0.127*cos(-pivotPos))
        + min(1.735*sin(-pivotPos),offset*sin(-pivotPos)),
        Rotation3d(0.0, -robot.wrist.positionSupplier.get() + pivotPos, 0.0))
    )
//      Pose3d(-0.136, 0.0, 0.245,
//        Rotation3d(0.0, pivotPos, 0.0)),
//      // first stage max: 0.60
//      Pose3d(-0.136 + offset*cos(-pivotPos),
//        0.0,
//        0.245 + offset*cos(-pivotPos),
//        Rotation3d(0.0, pivotPos, 0.0)),
//      // second stage max : 0.575 (1.175)
//      Pose3d(-0.136 + offset*cos(-pivotPos),
//        0.0,
//        0.245 + offset*cos(-pivotPos),
//        Rotation3d(0.0, pivotPos, 0.0)),
//      // third stage max: 0.56 (1.735)
//      Pose3d(-0.136 + offset*cos(-pivotPos),
//        0.0,
//        0.245 + offset*cos(-pivotPos),
//        Rotation3d(0.0, pivotPos, 0.0)),
////      Pose3d(0.569, 0.0, 0.366,
////        Rotation3d(0.0, -robot.wrist.positionSupplier.get(), 0.0))
//      Pose3d(sqrt(0.569.pow(2) + 0.366.pow(2))*cos(-pivotPos + atan(0.569)),
//        0.0,
//        sqrt(0.569.pow(2) + 0.366.pow(2))*sin(-pivotPos + atan(0.366)),
//        Rotation3d(0.0, -robot.wrist.positionSupplier.get(), 0.0))
//    )
    //sqrt(x0^2+y0^2)*<cos, sin>(theta + atan(y0, x0))
    DogLog.log("SimComponent", componentStorage)
    DogLog.log("offset", offset)

    robot.drive as SwerveSim

    VisionConstants.ESTIMATORS.forEach {
      it.simulationPeriodic(robot.drive.odometryPose)
    }

    VisionConstants.VISION_SIM.debugField.getObject("EstimatedRobot").pose = robot.poseSubsystem.pose

    // change elevator angle according to pivot position
    robot.elevator.elevatorSim?.changeAngle(robot.pivot.positionSupplier.get())
  }
}
