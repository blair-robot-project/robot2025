package frc.team449.auto

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.Robot
import frc.team449.auto.routines.DoNothing

class RoutineChooser(private val robot: Robot) : SendableChooser<String>() {

  fun routineMap(): HashMap<String, Command> {
    return hashMapOf(
      //jjj
    )
  }

  fun createOptions() {
    /** Add auto options here */
    this.setDefaultOption("Do Nothing", "DoNothing")

    this.addOption("Red Stage 3 L4", "RedStageThreeL4")
    this.addOption("Blue Stage 3 L4", "BlueStageThreeL4")
  }
}
