package frc.team449

import com.ctre.phoenix6.SignalLogger
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import dev.doglog.DogLog
import dev.doglog.DogLogOptions
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.auto.RoutineChooser
import frc.team449.commands.light.BlairChasing
import frc.team449.commands.light.BreatheHue
import frc.team449.commands.light.Rainbow
import frc.team449.subsystems.FieldConstants
import frc.team449.subsystems.drive.swerve.SwerveSim
import frc.team449.subsystems.vision.VisionConstants
import frc.team449.system.encoder.QuadCalibration
import org.littletonrobotics.urcl.URCL
import kotlin.jvm.optionals.getOrDefault
import kotlin.jvm.optionals.getOrNull

/** The main class of the robot, constructs all the subsystems
 * and initializes default commands . */
class RobotLoop : TimedRobot() {

  private val robot = Robot()

  private val routineChooser: RoutineChooser = RoutineChooser(robot)

  private var autoCommand: Command? = null
  private var routineMap = hashMapOf<String, Command>()
  private val controllerBinder = ControllerBindings(robot.driveController, robot.mechController, robot)

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
        .withCaptureDs(false)
        .withCaptureNt(false)
        .withLogExtras(false)
    )

    DogLog.setPdh(robot.powerDistribution)

    SmartDashboard.putData("Field", robot.field)

    URCL.start()

    val motor1 = SparkMax(2, SparkLowLevel.MotorType.kBrushless)
    val motor2 = SparkMax(62, SparkLowLevel.MotorType.kBrushless)

    motor1.setVoltage(6.0)
    motor2.setVoltage(6.0)
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

  override fun simulationInit() {}

  override fun simulationPeriodic() {
    robot.drive as SwerveSim

    VisionConstants.ESTIMATORS.forEach {
      it.simulationPeriodic(robot.drive.odometryPose)
    }

    VisionConstants.VISION_SIM.debugField.getObject("EstimatedRobot").pose = robot.poseSubsystem.pose

    // change elevator angle according to pivot position
  }
}
