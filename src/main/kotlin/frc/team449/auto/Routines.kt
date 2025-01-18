package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
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

  fun Shootl4Routine(): AutoRoutine {
    val l4Routine: AutoRoutine = autoFactory.newRoutine("L4 Routine")

    val l4fTrajectory: AutoTrajectory = l4Routine.trajectory("Go to l4F")
    val rightStationTrajectory: AutoTrajectory = l4Routine.trajectory("Go To Right Station ")
    val l1CTrajectory: AutoTrajectory = l4Routine.trajectory("Go to L1C")

    l4Routine.active().onTrue(
      Commands.sequence(
        l4fTrajectory.resetOdometry(),
        Commands.parallel(
          l4fTrajectory.cmd(),
          PrintCommand("Traveling to l4")
        ),
        l4fTrajectory.done().run {
          PrintCommand("We are Shooting!")
        },


        rightStationTrajectory.resetOdometry(),
        Commands.parallel(
          rightStationTrajectory.cmd(),
          PrintCommand("Intake Coral")
        ),
        l1CTrajectory.resetOdometry(),
        Commands.parallel(
          l1CTrajectory.cmd(),
          PrintCommand("SHOOT!")
        )
      )
    )

    return l4Routine
  }











  fun addOptions(autoChooser: AutoChooser){
    autoChooser.addRoutine("Shoot l4",this::Shootl4Routine)


  }
}