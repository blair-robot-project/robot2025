package frc.team449.auto.routines

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.Robot
import frc.team449.auto.choreo.ChoreoRoutine
import frc.team449.auto.choreo.ChoreoRoutineStructure
import frc.team449.auto.choreo.ChoreoTrajectory

class DoNothing(private val robot: Robot) : ChoreoRoutineStructure {

    override val routine = ChoreoRoutine(drive = robot.drive, poseSubsystem = robot.poseSubsystem)

    override val trajectory: MutableList<ChoreoTrajectory> = mutableListOf()

    override fun createCommand(): Command {
        return InstantCommand()
    }
}
