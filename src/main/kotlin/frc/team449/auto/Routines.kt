package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.team449.Robot
import frc.team449.RobotContainer

class Routines(
  robot: RobotContainer
) {
  val autoFactory = AutoFactory(
    robot.drive::pose,
    robot.drive::resetOdometry,
    robot.drive::followTrajectory,
    true,
    robot.drive
  )

  fun doNothing(): Command {
    return Commands.sequence()
  }

  fun inTake(): Command{
    return Commands.sequence(

    )
  }

  fun addOptions(autoChooser: AutoChooser) {
    autoChooser.addCmd("Do Nothing", this::doNothing)
  }
}