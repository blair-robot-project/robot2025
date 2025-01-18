package frc.team449

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import frc.team449.auto.Routines
import frc.team449.commands.light.BlairChasing
import frc.team449.commands.light.Rainbow
import frc.team449.subsystems.drive.swerve.SwerveSim
import frc.team449.subsystems.elevator.ElevatorConstants
import frc.team449.subsystems.elevator.ElevatorFeedForward.Companion.createElevatorFeedForward
import frc.team449.subsystems.pivot.PivotFeedForward.Companion.createPivotFeedForward
import frc.team449.subsystems.vision.VisionConstants
import monologue.Annotations.Log
import monologue.Logged
import monologue.Monologue
import org.littletonrobotics.urcl.URCL
import kotlin.math.PI

/** The main class of the robot, constructs all the subsystems
 * and initializes default commands . */
class RobotLoop : TimedRobot(), Logged {

  @Log.NT
  private val robot = Robot()

  @Log.NT
  private val field = robot.field

  private val controllerBinder = ControllerBindings(robot.driveController, robot.mechController, robot)

  override fun robotInit() {
    // Yes this should be a print statement, it's useful to know that robotInit started.
    println("Started robotInit.")

    SignalLogger.setPath("/media/sda1/ctre-logs/")
    SignalLogger.start()

    HAL.report(FRCNetComm.tResourceType.kResourceType_Language, FRCNetComm.tInstances.kLanguage_Kotlin)

    if (RobotBase.isSimulation()) {
      // Don't complain about joysticks if there aren't going to be any
      DriverStation.silenceJoystickConnectionWarning(true)
//      val instance = NetworkTableInstance.getDefault()
//      instance.stopServer()
//      instance.startClient4("localhost")
    }

    robot.elevator.elevatorFeedForward = createElevatorFeedForward(robot.pivot)
    robot.pivot.pivotFeedForward = createPivotFeedForward(robot.elevator)

    println("Generating Auto Routines : ${Timer.getFPGATimestamp()}")
    val routines = Routines(robot)

    routines.addOptions(robot.autoChooser)

    println("Putting the thing on the other thing")
    SmartDashboard.putData("Auto Chooser", robot.autoChooser)

    RobotModeTriggers.autonomous().whileTrue(robot.autoChooser.selectedCommandScheduler())
    println("DONE Generating Auto Routines : ${Timer.getFPGATimestamp()}")

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance())

    robot.light.defaultCommand = BlairChasing(robot.light)

    controllerBinder.bindButtons()

    DriverStation.startDataLog(DataLogManager.getLog())
    Monologue.setupMonologue(this, "/Monologuing", false, false)

    URCL.start()
  }

  override fun driverStationConnected() {
    Monologue.setFileOnly(DriverStation.isFMSAttached())
  }

  override fun robotPeriodic() {
    CommandScheduler.getInstance().run()

    // Robot Drive Logging
    robot.field.robotPose = robot.poseSubsystem.pose
    robot.field.getObject("bumpers").pose = robot.poseSubsystem.pose

    // Superstructure Simulation
    robot.elevator.elevatorLigament.length = ElevatorConstants.MIN_VIS_HEIGHT + robot.elevator.positionSupplier.get()
    robot.elevator.desiredElevatorLigament.length = ElevatorConstants.MIN_VIS_HEIGHT + robot.elevator.targetSupplier.get()

    robot.elevator.elevatorLigament.angle = robot.pivot.positionSupplier.get() * (180 / PI)
    robot.elevator.desiredElevatorLigament.angle = robot.pivot.targetSupplier.get() * (180 / PI)

    robot.elevator.wristLigament.angle = robot.wrist.positionSupplier.get() * (180 / PI)

    SmartDashboard.putData("Elevator + Pivot Visual", robot.elevator.mech)

    // Monologue Logging
    Monologue.updateAll()
  }

  override fun autonomousInit() {}

  override fun autonomousPeriodic() {}

  override fun teleopInit() {
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

  override fun testInit() {}

  override fun testPeriodic() {}

  override fun simulationInit() {}

  override fun simulationPeriodic() {
    robot.drive as SwerveSim

    VisionConstants.ESTIMATORS.forEach {
      it.simulationPeriodic(robot.drive.odometryPose)
    }

    VisionConstants.VISION_SIM.debugField.getObject("EstimatedRobot").pose = robot.poseSubsystem.pose

    // change elevator angle according to pivot position
    robot.elevator.elevatorSim?.changeAngle(robot.pivot.positionSupplier.get())
  }
}
