package frc.team449

import com.ctre.phoenix6.SignalLogger
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.*
import frc.team449.auto.RoutineChooser
import frc.team449.commands.autoscoreCommands.AutoScoreCommandConstants
import frc.team449.commands.autoscoreCommands.AutoScoreCommands
import frc.team449.commands.autoscoreCommands.WebConnection
import frc.team449.commands.light.BlairChasing
import frc.team449.commands.light.BreatheHue
import frc.team449.commands.light.Rainbow
import frc.team449.subsystems.drive.swerve.SwerveConstants
import frc.team449.subsystems.drive.swerve.SwerveSim
import frc.team449.subsystems.elevator.ElevatorFeedForward.Companion.createElevatorFeedForward
import frc.team449.subsystems.pivot.PivotFeedForward.Companion.createPivotFeedForward
import frc.team449.subsystems.vision.VisionConstants
import monologue.Annotations.Log
import monologue.Logged
import monologue.Monologue
import org.littletonrobotics.urcl.URCL
import kotlin.jvm.optionals.getOrNull

/** The main class of the robot, constructs all the subsystems
 * and initializes default commands . */
class RobotLoop : TimedRobot(), Logged {
  @Log.NT
  val robot = Robot()

  private val routineChooser: RoutineChooser = RoutineChooser(robot)

  @Log.NT
  private val field = robot.field
  private var autoCommand: Command? = null
  private var routineMap = hashMapOf<String, Command>()
  private val controllerBinder = ControllerBindings(robot.driveController, robot.mechController, robot)
  private var webCom: WebConnection? = null
  private val autoscore = AutoScoreCommands(robot.drive, robot.poseSubsystem, robot.driveController.hid, robot)

  override fun robotInit() {
    // Yes this should be a print statement, it's useful to know that robotInit started.
    println("Started robotInit.")

    SignalLogger.setPath("/media/sda1/ctre-logs/")
    SignalLogger.start()

    HAL.report(FRCNetComm.tResourceType.kResourceType_Language, FRCNetComm.tInstances.kLanguage_Kotlin)

    if (RobotBase.isSimulation()) {
      // Don't complain about joysticks if there aren't going to be any
      DriverStation.silenceJoystickConnectionWarning(true)
    }

    robot.elevator.elevatorFeedForward = createElevatorFeedForward(robot.pivot)
    robot.pivot.pivotFeedForward = createPivotFeedForward(robot.elevator)

    /** Example Quad Calibration
     QuadCalibration(robot.pivot).ignoringDisable(true).schedule()
     */

    println("Generating Auto Routines : ${Timer.getFPGATimestamp()}")
    routineMap = routineChooser.routineMap()
    println("DONE Generating Auto Routines : ${Timer.getFPGATimestamp()}")

    routineChooser.createOptions()

    SmartDashboard.putData("Routine Chooser", routineChooser)
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance())

    robot.light.defaultCommand = BlairChasing(robot.light)

    DriverStation.startDataLog(DataLogManager.getLog())
    Monologue.setupMonologue(this, "/Monologuing", false, false)

    URCL.start()
  }

  override fun driverStationConnected() {
    controllerBinder.bindButtons()

    Monologue.setFileOnly(DriverStation.isFMSAttached())

    webCom = WebConnection()
  }

  override fun robotPeriodic() {
    CommandScheduler.getInstance().run()

    robot.field.robotPose = robot.poseSubsystem.pose

    robot.field.getObject("bumpers").pose = robot.poseSubsystem.pose

    Monologue.updateAll()
  }

  override fun autonomousInit() {
    /** Every time auto starts, we update the chosen auto command. */
    this.autoCommand = routineMap[if (DriverStation.getAlliance().getOrNull() == Alliance.Red) "Red" + routineChooser.selected else "Blue" + routineChooser.selected]
    CommandScheduler.getInstance().schedule(this.autoCommand)

    if (DriverStation.getAlliance().getOrNull() == Alliance.Red) {
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
    webCom?.closeServer()
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
    webCom?.command = webCom?.commandSubscriber?.get().toString()

    if(webCom?.command != "none" && DriverStation.isDSAttached()) {
      println("command received")
      webCom?.isDonePublish?.set(false)

      var command = autoscore.processor()
      //get the value
      when (webCom?.command) {
        "processor" -> command = autoscore.processor()
        "intakeCoralTop" -> command = autoscore.coral(true)
        "intakeCoralBottom" -> command = autoscore.coral(false)
        "netRed" -> command = autoscore.net(true)
        "netBlue" -> command = autoscore.net(false)
        else -> {
          //format will be l_ location__
          val level = webCom?.command?.slice(0..1)
          val location = webCom?.command?.slice(3..<webCom?.command!!.length)
          var reefLocation = AutoScoreCommandConstants.ReefLocation.Location1
          var reefLevel = (AutoScoreCommandConstants.ReefLevel.L1)
          when(location) {
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
          when(level) {
            "l1" -> reefLevel = (AutoScoreCommandConstants.ReefLevel.L1)
            "l2" -> reefLevel = (AutoScoreCommandConstants.ReefLevel.L2)
            "l3" -> reefLevel = (AutoScoreCommandConstants.ReefLevel.L3)
            "l4" -> reefLevel = (AutoScoreCommandConstants.ReefLevel.L4)
          }
          command = autoscore.reef(reefLocation, reefLevel)
        }
      }
      command.schedule()
      webCom?.isDonePublish?.set(true)
      webCom?.commandPublisher?.set("none")
    }
  }
}
