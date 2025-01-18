package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import edu.wpi.first.wpilibj2.command.Commands
import frc.team449.Robot

class Routines(
  robot: Robot
) {
  val autoFactory = AutoFactory(
    robot.poseSubsystem::pose,
    robot.poseSubsystem::resetOdometry,
    robot.drive::followTrajectory,
    true,
    robot.drive
  )


  fun coralL1reefE(): AutoRoutine {
    val l1reefE: AutoRoutine = autoFactory.newRoutine("coral L1 reef E")
    val reefETrajectory: AutoTrajectory = l1reefE.trajectory("right_E")
    l1reefE.active().onTrue(
      Commands.sequence(
        reefETrajectory.resetOdometry(),
        reefETrajectory.cmd()
      )
    )
    return l1reefE
  }

  fun addOptions(autoChooser: AutoChooser) {
    autoChooser.addRoutine("l1reefE", this::coralL1reefE)
  }
}
