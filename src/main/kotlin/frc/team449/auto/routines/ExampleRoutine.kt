package frc.team449.auto.routines

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.team449.Robot
import frc.team449.auto.AutoUtil
import frc.team449.auto.choreo.ChoreoRoutine
import frc.team449.auto.choreo.ChoreoRoutineStructure
import frc.team449.auto.choreo.ChoreoTrajectory

class ExampleRoutine(
  robot: Robot,
  isRed: Boolean
) : ChoreoRoutineStructure {

  override val routine =
    ChoreoRoutine(
      drive = robot.drive,
      poseSubsystem = robot.poseSubsystem,
      parallelEventMap = hashMapOf(
        0 to InstantCommand(),
        1 to InstantCommand()
      ),
      stopEventMap = hashMapOf(
        0 to PrintCommand("Going to start moving!"),
        1 to PrintCommand("Finished the 1st trajectory!"),
        2 to PrintCommand("Finished the routine!")
      ),
      debug = false,
      timeout = 1.5
    )

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRed) {
      AutoUtil.transformForRedAlliance(
        ChoreoTrajectory.createTrajectory(arrayListOf("part1", "part2"), "example")
      )
    } else {
      ChoreoTrajectory.createTrajectory(arrayListOf("part1", "part2"), "example")
    }
}
