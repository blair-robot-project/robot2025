package frc.team449.auto

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.Robot
import frc.team449.auto.routines.DoNothing

class RoutineChooser(private val robot: Robot) : SendableChooser<String>() {

    fun routineMap(): HashMap<String, Command> {
        return hashMapOf(
            "RedDoNothing" to DoNothing(robot).createCommand(),
            "BlueDoNothing" to DoNothing(robot).createCommand(),
        )
    }

    fun createOptions() {
        /** Add auto options here */
        this.setDefaultOption("Do Nothing", "DoNothing")
    }
}
