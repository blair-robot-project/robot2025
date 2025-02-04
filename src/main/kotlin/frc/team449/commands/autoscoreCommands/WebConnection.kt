package frc.team449.commands.autoscoreCommands

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StringPublisher
import edu.wpi.first.networktables.StringSubscriber
import edu.wpi.first.wpilibj.DriverStation

class WebConnection : Runnable {
  private val instance = NetworkTableInstance.getDefault()
  private val webComTable = instance.getTable("webcom")
  private val allianceTopic = webComTable.getStringTopic("Alliance")
  private val isDoneTopic = webComTable.getBooleanTopic("isDone")
  val commandSubscriber: StringSubscriber
  val commandPublisher: StringPublisher
  val isDonePublish = isDoneTopic.publish()
  private val alliancePublish = allianceTopic.publish()
  var command = "none"

  init {
    instance.startClient4("localhost")
    instance.setServerTeam(449)
    val alliance = if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) "Red" else "Blue"
    alliancePublish.set(alliance)
    isDonePublish.set(true)
    // have to set command here because topics will not be created yet.
    commandSubscriber = webComTable.getStringTopic("Command").subscribe("none")
    commandPublisher = webComTable.getStringTopic("Command").publish()
    commandPublisher.set("none")
  }

  fun closeServer() {
    instance.stopServer()
    instance.close()
  }

  override fun run() {
    commandPublisher.set("none")
    isDonePublish.set(true)
  }
}
