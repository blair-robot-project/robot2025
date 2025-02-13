package frc.team449.auto

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.Robot
import frc.team449.auto.routines.DoNothing
import frc.team449.auto.routines.ThreeL4

class RoutineChooser(private val robot: Robot) : SendableChooser<String>() {

  fun routineMap(): HashMap<String, Command> {
    return hashMapOf(
      "RedDoNothing" to DoNothing(robot).createCommand(),
      "BlueDoNothing" to DoNothing(robot).createCommand(),
      "RedRedStageThreeL4" to ThreeL4(robot, isRedAlliance = true, isRedStage = true).createCommand(),
      "RedBlueStageThreeL4" to ThreeL4(robot, isRedAlliance = true, isRedStage = false).createCommand(),
      "BlueRedStageThreeL4" to ThreeL4(robot, isRedAlliance = false, isRedStage = true).createCommand(),
      "BlueBlueStageThreeL4" to ThreeL4(robot, isRedAlliance = false, isRedStage = false).createCommand(),
    )
  }

  fun createOptions() {
    /** Add auto options here */
    this.setDefaultOption("Do Nothing", "DoNothing")

    this.addOption("Red Stage 3 L4", "RedStageThreeL4")
    this.addOption("Blue Stage 3 L4", "BlueStageThreeL4")
  }
}
