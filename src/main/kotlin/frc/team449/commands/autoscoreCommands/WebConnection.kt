package frc.team449.commands.autoscoreCommands

import edu.wpi.first.networktables.NetworkTableInstance

class WebConnection {
  private val instance = NetworkTableInstance.getDefault()
  val WebComTable = instance.getTable("webcom")

  init {
    instance.stopServer()
    instance.startClient4("localhost")
  }

}