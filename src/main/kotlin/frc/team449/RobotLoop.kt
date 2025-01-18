package frc.team449

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.auto.RoutineChooser
import frc.team449.commands.autoscoreCommands.AutoScoreCommandConstants
import frc.team449.commands.autoscoreCommands.AutoScoreCommands
import frc.team449.commands.autoscoreCommands.WebConnection
import frc.team449.commands.light.BlairChasing
import frc.team449.commands.light.BreatheHue
import frc.team449.commands.light.Rainbow
import frc.team449.subsystems.drive.swerve.SwerveSim
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
  private val robot = Robot()

  private val routineChooser: RoutineChooser = RoutineChooser(robot)

  @Log.NT
  private val field = robot.field
  private var autoCommand: Command? = null
  private var routineMap = hashMapOf<String, Command>()
  private val controllerBinder = ControllerBindings(robot.driveController, robot.mechController, robot)
  private var webCom: WebConnection? = null
  private val autoscore = AutoScoreCommands(robot.drive, robot.poseSubsystem, robot.driveController.hid)

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


    controllerBinder.bindButtons()

    DriverStation.startDataLog(DataLogManager.getLog())
    Monologue.setupMonologue(this, "/Monologuing", false, false)

    URCL.start()
  }

  override fun driverStationConnected() {
    Monologue.setFileOnly(DriverStation.isFMSAttached())
    // temporary bindings for sim testing
    robot.driveController.x().onTrue(
      autoscore.moveToReefCommand(AutoScoreCommandConstants.ReefLocation.Location1)
        .andThen(autoscore.putCoralInReef(AutoScoreCommandConstants.ReefLevel.L1))
    )
    robot.driveController.a().onTrue(
      autoscore.moveToProcessorCommand().andThen(autoscore.scoreProcessorCommand())
    )
    robot.driveController.b().onTrue(
      autoscore.moveToNetCommand(false)
        .andThen(autoscore.scoreNetCommand())
    )
    robot.driveController.y().onTrue(
      autoscore.moveToCoralIntakeCommand(true)
        .andThen(autoscore.intakeCoralCommand())
    )
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

      var command = autoscore.moveToProcessorCommand().andThen(autoscore.scoreProcessorCommand())
      var reef = false
      //get the value
      when (webCom?.command) {
        "processor" -> command = autoscore.moveToProcessorCommand().andThen(autoscore.scoreProcessorCommand())
        "intakeCoralTop" -> command = autoscore.moveToCoralIntakeCommand(true).andThen(autoscore.intakeCoralCommand())
        "intakeCoralBottom" -> command = autoscore.moveToCoralIntakeCommand(false).andThen(autoscore.intakeCoralCommand())
        "netRed" -> command = autoscore.moveToNetCommand(true).andThen(autoscore.scoreNetCommand())
        "netBlue" -> command = autoscore.moveToNetCommand(false).andThen(autoscore.scoreNetCommand())
        else -> {
          reef = true
          //format will be l_ location__
          val level = webCom?.command?.slice(0..1)
          val location = webCom?.command?.slice(3..<webCom?.command!!.length)
          var firstCommand = autoscore.moveToReefCommand(AutoScoreCommandConstants.ReefLocation.Location1)
          when(location) {
            "location1" -> firstCommand = autoscore.moveToReefCommand(AutoScoreCommandConstants.ReefLocation.Location1)
            "location2" -> firstCommand = autoscore.moveToReefCommand(AutoScoreCommandConstants.ReefLocation.Location2)
            "location3" -> firstCommand = autoscore.moveToReefCommand(AutoScoreCommandConstants.ReefLocation.Location3)
            "location4" -> firstCommand = autoscore.moveToReefCommand(AutoScoreCommandConstants.ReefLocation.Location4)
            "location5" -> firstCommand = autoscore.moveToReefCommand(AutoScoreCommandConstants.ReefLocation.Location5)
            "location6" -> firstCommand = autoscore.moveToReefCommand(AutoScoreCommandConstants.ReefLocation.Location6)
            "location7" -> firstCommand = autoscore.moveToReefCommand(AutoScoreCommandConstants.ReefLocation.Location7)
            "location8" -> firstCommand = autoscore.moveToReefCommand(AutoScoreCommandConstants.ReefLocation.Location8)
            "location9" -> firstCommand = autoscore.moveToReefCommand(AutoScoreCommandConstants.ReefLocation.Location9)
            "location10" -> firstCommand = autoscore.moveToReefCommand(AutoScoreCommandConstants.ReefLocation.Location10)
            "location11" -> firstCommand = autoscore.moveToReefCommand(AutoScoreCommandConstants.ReefLocation.Location11)
            "location12" -> firstCommand = autoscore.moveToReefCommand(AutoScoreCommandConstants.ReefLocation.Location12)
          }
          var secondCommand = autoscore.moveToReefCommand(AutoScoreCommandConstants.ReefLocation.Location1)
          when(level) {
            "l1" -> secondCommand = autoscore.putCoralInReef(AutoScoreCommandConstants.ReefLevel.L1)
            "l2" -> secondCommand = autoscore.putCoralInReef(AutoScoreCommandConstants.ReefLevel.L2)
            "l3" -> secondCommand = autoscore.putCoralInReef(AutoScoreCommandConstants.ReefLevel.L3)
            "l4" -> secondCommand = autoscore.putCoralInReef(AutoScoreCommandConstants.ReefLevel.L4)
          }
          println("running reef jaunt")
          firstCommand.andThen(secondCommand).schedule()
          //
        }
      }
      if(!reef) {
        println("running non reef jaunt")
        command.schedule()
      }
      webCom?.isDonePublish?.set(true)
      webCom?.commandPublisher?.set("none")
    }
  }
}
