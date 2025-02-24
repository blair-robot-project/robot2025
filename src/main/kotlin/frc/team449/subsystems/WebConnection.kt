package frc.team449.subsystems

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StringPublisher
import edu.wpi.first.networktables.StringSubscriber
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Robot
import frc.team449.commands.autoscoreCommands.AutoScoreCommandConstants
import frc.team449.commands.autoscoreCommands.AutoScoreCommands

class WebConnection(val robot: Robot) : SubsystemBase() {
  private val instance = NetworkTableInstance.getDefault()
  private val webComTable = instance.getTable("webcom")
  private val allianceTopic = webComTable.getStringTopic("Alliance")
  private val isDoneTopic = webComTable.getBooleanTopic("isDone")
  private val commandSubscriber: StringSubscriber = webComTable.getStringTopic("Command").subscribe("none")
  private val commandPublisher: StringPublisher = webComTable.getStringTopic("Command").publish()
  private val isDonePublish = isDoneTopic.publish()
  private val isDoneSub = isDoneTopic.subscribe(true)
  private val alliancePublish = allianceTopic.publish()
  private var ntCommandInput = "none"
  private var autoScore = AutoScoreCommands(robot)
  private var webAppCommand : Command = InstantCommand()

  fun setUpNT() {
    instance.startClient4("localhost")
    instance.setServerTeam(449)
    isDonePublish.set(true)
    commandPublisher.set("none")
    val alliance = if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) "Red" else "Blue"
    alliancePublish.set(alliance)
  }

  fun closeServer() {
    instance.stopServer()
    instance.close()
  }

  override fun periodic() {
    ntCommandInput = commandSubscriber.get().toString()

    if (ntCommandInput != "none" && DriverStation.isDSAttached()) {
      println("command received")
      isDonePublish.set(false)

      webAppCommand = when (ntCommandInput) {
        "processor" -> autoScore.getProcessorCommand()
        "intakeCoralTop" -> InstantCommand()
        "intakeCoralBottom" -> InstantCommand()
        "netRed" -> autoScore.getNetCommand(true)
        "netBlue" -> autoScore.getNetCommand(false)
        "cancel" -> autoScore.cancelCommand()
        else -> {
          //format will be l_ location__
          val level = ntCommandInput.slice(0..1)
          val location = ntCommandInput.slice(3..<ntCommandInput.length)
          val reefLocation = when (location) {
            "location1" -> (AutoScoreCommandConstants.ReefLocation.Location1)
            "location2" -> (AutoScoreCommandConstants.ReefLocation.Location2)
            "location3" -> (AutoScoreCommandConstants.ReefLocation.Location3)
            "location4" -> (AutoScoreCommandConstants.ReefLocation.Location4)
            "location5" -> (AutoScoreCommandConstants.ReefLocation.Location5)
            "location6" -> (AutoScoreCommandConstants.ReefLocation.Location6)
            "location7" -> (AutoScoreCommandConstants.ReefLocation.Location7)
            "location8" -> (AutoScoreCommandConstants.ReefLocation.Location8)
            "location9" -> (AutoScoreCommandConstants.ReefLocation.Location9)
            "location10" -> (AutoScoreCommandConstants.ReefLocation.Location10)
            "location11" -> (AutoScoreCommandConstants.ReefLocation.Location11)
            "location12" -> (AutoScoreCommandConstants.ReefLocation.Location12)
            else -> (AutoScoreCommandConstants.ReefLocation.Location1)
          }
          val coralLevel = when (level) {
            "l1" -> (AutoScoreCommandConstants.CoralLevel.L1)
            "l2" -> (AutoScoreCommandConstants.CoralLevel.L2)
            "l3" -> (AutoScoreCommandConstants.CoralLevel.L3)
            "l4" -> (AutoScoreCommandConstants.CoralLevel.L4)
            else -> (AutoScoreCommandConstants.CoralLevel.L1)
          }
          autoScore.getReefCommand(reefLocation, coralLevel)
        }
      }
      webAppCommand.schedule()
      isDonePublish.set(false)
      commandPublisher.set("none")
    } else {
      if(!isDoneSub.get()) {
        if(!webAppCommand.isScheduled) {
          isDonePublish.set(true)
        }
      }
    }


  }
}
