package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.team449.Robot

class Routines(
  robot: Robot

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


  fun Shootl4(): Command{
     return Commands.sequence(
       autoFactory.resetOdometry("Go to l4F"),
       Commands.parallel(
         autoFactory.trajectoryCmd("Go to l4F"),
         PrintCommand("Going to l4f")
     ),
       PrintCommand("Shoot l4"),

    )
  }

  fun goToRightStation():Command{
    return Commands.sequence(
      autoFactory.resetOdometry("Go To Right Station"),
      Commands.parallel(
        autoFactory.trajectoryCmd("Go To Right Station"),
        PrintCommand("Traveling to Right Station"),
      ),
      PrintCommand("Intake Coral")
    )


  }


  fun toL1C():Command{
    return Commands.sequence(
      autoFactory.resetOdometry("Go to L1C"),
      Commands.parallel(
        autoFactory.trajectoryCmd("Go to L1C"),
        PrintCommand("Traveling to L1C")
      ),
      PrintCommand("Shoot Coral")
    )

  }

  fun addOptions(autoChooser: AutoChooser) {
    autoChooser.addCmd("Do Nothing", this::doNothing)
  }
}