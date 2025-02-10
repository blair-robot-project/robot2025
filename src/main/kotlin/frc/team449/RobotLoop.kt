package frc.team449

import com.ctre.phoenix6.SignalLogger
import com.pathplanner.lib.pathfinding.Pathfinding
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
import frc.team449.subsystems.superstructure.elevator.ElevatorConstants
import frc.team449.subsystems.superstructure.elevator.ElevatorFeedForward.Companion.createElevatorFeedForward
import frc.team449.subsystems.superstructure.pivot.PivotFeedForward.Companion.createPivotFeedForward
import frc.team449.subsystems.vision.VisionConstants
import frc.team449.system.encoder.QuadCalibration
import org.littletonrobotics.urcl.URCL
import kotlin.jvm.optionals.getOrDefault
import kotlin.jvm.optionals.getOrNull
import frc.team449.commands.autoscoreCommands.AutoScoreCommands
import frc.team449.commands.autoscoreCommands.AutoScoreCommandConstants
import frc.team449.commands.autoscoreCommands.WebConnection


/** The main class of the robot, constructs all the subsystems
 * and initializes default commands . */
class RobotLoop : TimedRobot() {
  val robot = Robot()

  private val routineChooser: RoutineChooser = RoutineChooser(robot)

  private val field = robot.field
  private var autoCommand: Command? = null
  private var routineMap = hashMapOf<String, Command>()
  private val controllerBinder = ControllerBindings(robot.driveController, robot.mechController, robot)

  private val autoScore = AutoScoreCommands(
    robot.poseSubsystem,
    robot
  )
  private var webCom : WebConnection? = null

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

    DriverStation.startDataLog(DataLogManager.getLog())

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
    webCom = WebConnection()
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
    Pathfinding.setPathfinder(robot.pathfinder.ADStar)
  }
  override fun simulationPeriodic() {
    robot.drive as SwerveSim

    VisionConstants.ESTIMATORS.forEach {
      it.simulationPeriodic(robot.poseSubsystem.pose)
    }

    VisionConstants.VISION_SIM.debugField.getObject("EstimatedRobot").pose = robot.poseSubsystem.pose

    // change elevator angle according to pivot position
    robot.elevator.elevatorSim?.changeAngle(robot.pivot.positionSupplier.get())

    webCom?.command = webCom?.commandSubscriber?.get().toString()

    if (webCom?.command != "none" && DriverStation.isDSAttached()) {
      println("command received")
      webCom?.isDonePublish?.set(false)

      val command : Command
      //get the value
      when (webCom?.command) {
        "processor" -> command = autoScore.processor()
        "intakeCoralTop" -> command = autoScore.coral(true)
        "intakeCoralBottom" -> command = autoScore.coral(false)
        "netRed" -> command = autoScore.net(true)
        "netBlue" -> command = autoScore.net(false)
        else -> {
          //format will be l_ location__
          val level = webCom?.command?.slice(0..1)
          val location = webCom?.command?.slice(3..<webCom?.command!!.length)
          var reefLocation = AutoScoreCommandConstants.ReefLocation.Location1
          var reefLevel = (AutoScoreCommandConstants.ReefLevel.L1)
          when (location) {
            "location1" -> reefLocation = (AutoScoreCommandConstants.ReefLocation.Location1)
            "location2" -> reefLocation = (AutoScoreCommandConstants.ReefLocation.Location2)
            "location3" -> reefLocation = (AutoScoreCommandConstants.ReefLocation.Location3)
            "location4" -> reefLocation = (AutoScoreCommandConstants.ReefLocation.Location4)
            "location5" -> reefLocation = (AutoScoreCommandConstants.ReefLocation.Location5)
            "location6" -> reefLocation = (AutoScoreCommandConstants.ReefLocation.Location6)
            "location7" -> reefLocation = (AutoScoreCommandConstants.ReefLocation.Location7)
            "location8" -> reefLocation = (AutoScoreCommandConstants.ReefLocation.Location8)
            "location9" -> reefLocation = (AutoScoreCommandConstants.ReefLocation.Location9)
            "location10" -> reefLocation = (AutoScoreCommandConstants.ReefLocation.Location10)
            "location11" -> reefLocation = (AutoScoreCommandConstants.ReefLocation.Location11)
            "location12" -> reefLocation = (AutoScoreCommandConstants.ReefLocation.Location12)
          }
          when (level) {
            "l1" -> reefLevel = (AutoScoreCommandConstants.ReefLevel.L1)
            "l2" -> reefLevel = (AutoScoreCommandConstants.ReefLevel.L2)
            "l3" -> reefLevel = (AutoScoreCommandConstants.ReefLevel.L3)
            "l4" -> reefLevel = (AutoScoreCommandConstants.ReefLevel.L4)
          }
          command = autoScore.reef(reefLocation, reefLevel)
        }
      }
      autoScore.currentCommand = command
      autoScore.poseSubsystem.autoscoreCurrentCommand = command
      command.schedule()
      webCom?.isDonePublish?.set(true)
      webCom?.commandPublisher?.set("none")
    }
  }
}
