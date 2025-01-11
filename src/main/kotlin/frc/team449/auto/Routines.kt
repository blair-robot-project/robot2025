package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.team449.Robot
import frc.team449.RobotContainer

class Routines(
  robot: RobotContainer

) {
  val autoFactory = AutoFactory(
    robot.poseSubsystem::pose,
    robot.poseSubsystem::resetOdometry,
    robot.drive::followTrajectory,
    true,
    robot.drive
  )

  fun doNothing(): Command {
    return Commands.sequence()
  }

  fun taxi(): Command{
    return Commands.sequence(
      autoFactory.resetOdometry("leftTaxi"),
      PrintCommand("resetting odometry and then moving forward"),
      autoFactory.trajectoryCmd("leftTaxi")
    )
  }

  fun l4reefE():Command{
    return Commands.sequence(
      autoFactory.resetOdometry("right_E"),
      Commands.parallel(
        autoFactory.trajectoryCmd("right_E"),
        PrintCommand("elevator extending"),
      ),
      PrintCommand("wait for a bit to align the coral"),
      PrintCommand("placing coral"),
    )
  }

  fun l1reefJ():Command{
    return Commands.sequence(
      autoFactory.resetOdometry("left_J"),
      autoFactory.trajectoryCmd("left_J"),
      PrintCommand("place coral in l1")
    )
  }

  fun l1reefG():Command{
    return Commands.sequence(
      autoFactory.resetOdometry("middle_G"),
      autoFactory.trajectoryCmd("middle_G"),
      PrintCommand("place coral in l1")
    )
  }

  fun l1reefF():Command{
    return Commands.sequence(
      autoFactory.resetOdometry("left_F"),
      autoFactory.trajectoryCmd("left_F"),
      PrintCommand("place coral in l1"))
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
    autoChooser.addCmd("Left Taxi", this::taxi)
    autoChooser.addCmd("L4 To reef E", this::l4reefE)
    autoChooser.addCmd("L1 To reef F", this::l1reefF)
    autoChooser.addCmd("L1 To reef G", this::l1reefG)
    autoChooser.addCmd("L1 To reef J", this::l1reefJ)
  }
}