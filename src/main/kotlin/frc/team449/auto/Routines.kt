package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import choreo.trajectory.SwerveSample
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.Robot
import frc.team449.commands.driveAlign.SimpleReefAlign
import frc.team449.subsystems.FieldConstants
import frc.team449.subsystems.superstructure.SuperstructureGoal
import java.util.Optional

open class Routines(
  val robot: Robot
) {

  val autoFactory = AutoFactory(
    robot.poseSubsystem::pose,
    robot.poseSubsystem::resetOdometry,
    { sample: SwerveSample -> robot.drive.followTrajectory(robot, sample) },
    true,
    robot.drive
  )

  // do nothing
  fun doNothing(): AutoRoutine {
    val nothing: AutoRoutine = autoFactory.newRoutine("Nothing")
    return nothing
  }

  fun americanRoutine(): AutoRoutine {
    val autoRoutine = autoFactory.newRoutine("L4 Routine")

    val l4fTrajectory = autoRoutine.trajectory("Go To L4F")
    val rightStationTrajectory = autoRoutine.trajectory("Go To Right Station(2)")
    val l1CTrajectory = autoRoutine.trajectory("Go To L1C")
    val rightStationTrajectory2 = autoRoutine.trajectory("Go To Right Station(Again)")
    val l1bTrajectory = autoRoutine.trajectory("Go To L1B")
    val leftStationTrajectory = autoRoutine.trajectory("Go To Left Station")
    val l1kTrajectory = autoRoutine.trajectory("Go To L4K")

    autoRoutine.active().onTrue(
      Commands.sequence(
        l4fTrajectory.resetOdometry(),
        l4fTrajectory.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE),
          PrintCommand("Traveling to L4")
        )
      )
    )

    l4fTrajectory.done().onTrue(
      Commands.sequence(
        robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
          .alongWith(SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.RIGHT))),
        robot.drive.driveStop(),
        robot.intake.outtakeCoral().andThen(WaitUntilCommand { !robot.intake.coralDetected() }),
        WaitCommand(0.15).onlyIf { RobotBase.isReal() },
        rightStationTrajectory.cmd()
      )
    )

    rightStationTrajectory.done().onTrue(
      Commands.sequence(
        robot.drive.driveStop(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
          .andThen(robot.intake.intakeCoral())
          .andThen(WaitUntilCommand { robot.intake.coralDetected() })
          .andThen(WaitCommand(0.15)),
        l1CTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE))
      )
    )

    l1CTrajectory.done().onTrue(
      Commands.sequence(
        robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
          .alongWith(SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.RIGHT))),
        robot.drive.driveStop(),
        robot.intake.outtakeCoral().andThen(WaitUntilCommand { !robot.intake.coralDetected() }),
        WaitCommand(0.15).onlyIf { RobotBase.isReal() },
        rightStationTrajectory2.cmd()
      )
    )

    rightStationTrajectory2.done().onTrue(
      Commands.sequence(
        PrintCommand("Trajectory complete"),
        robot.drive.driveStop(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
          .andThen(robot.intake.intakeCoral())
          .andThen(WaitUntilCommand { robot.intake.coralDetected() })
          .andThen(WaitCommand(0.15).onlyIf { RobotBase.isReal() }),
        l1bTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE))
      )
    )

    l1bTrajectory.done().onTrue(
      Commands.sequence(
        robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
          .alongWith(SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.LEFT))),
        robot.drive.driveStop(),
        robot.intake.outtakeCoral().andThen(WaitUntilCommand { !robot.intake.coralDetected() }),
        WaitCommand(0.15).onlyIf { RobotBase.isReal() },
        leftStationTrajectory.cmd()
      )
    )

    leftStationTrajectory.done().onTrue(
      Commands.sequence(
        robot.drive.driveStop(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
          .andThen(robot.intake.intakeCoral())
          .andThen(WaitUntilCommand { robot.intake.coralDetected() })
          .andThen(WaitCommand(0.15).onlyIf { RobotBase.isReal() }),
        l1kTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE))
      )
    )

    l1kTrajectory.done().onTrue(
      Commands.sequence(
        robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
          .alongWith(SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.LEFT))),
        robot.intake.outtakeCoral().andThen(WaitUntilCommand { !robot.intake.coralDetected() }),
        robot.drive.driveStop(),
        WaitCommand(0.15),
        robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
      )
    )

    return autoRoutine
  }

  /** link to starting position on the field: https://docs.google.com/document/d/1SOzIJDgJ0GRSVnNTcBhaFfltvHw0IjJTEUsAZbI2hW4/edit?usp=sharing  **/
  /** left and right are from the driver's pov **/

  // right taxi
  fun rightTaxi(): AutoRoutine {
    val rTaxi: AutoRoutine = autoFactory.newRoutine("Right Taxi")
    val rTaxiTrajectory: AutoTrajectory = rTaxi.trajectory("taxiRight")
    rTaxi.active().onTrue(Commands.sequence(rTaxiTrajectory.resetOdometry(), rTaxiTrajectory.cmd(), robot.drive.driveStop()))
    return rTaxi
  }

  // left taxi
  fun leftTaxi(): AutoRoutine {
    val lTaxi: AutoRoutine = autoFactory.newRoutine("Left Taxi")
    val lTaxiTrajectory: AutoTrajectory = lTaxi.trajectory("taxiLeft")
    lTaxi.active().onTrue(Commands.sequence(lTaxiTrajectory.resetOdometry(), lTaxiTrajectory.cmd(), robot.drive.driveStop()))
    return lTaxi
  }

  // L1 coral at reef G
  fun l1reefG(): AutoRoutine {
    val g: AutoRoutine = autoFactory.newRoutine("L1 reef G")
    val reefGTrajectory: AutoTrajectory = g.trajectory("middle_G")
    g.active().onTrue(
      Commands.sequence(
        reefGTrajectory.resetOdometry(),
        Commands.deadline(reefGTrajectory.cmd(), robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE)),
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.RIGHT)).alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L1)
        ),
        robot.drive.driveStop().alongWith(robot.intake.outtakeCoral().until { (!robot.intake.coralDetected()) })
      )
    )
    return g
  }

  // l4 at reef E reef D
  fun reefED(): AutoRoutine {
    val E_D: AutoRoutine = autoFactory.newRoutine("L4 reef E reef D")
    val reefETrajectory: AutoTrajectory = E_D.trajectory("right_E")
    val reefEtoStationTrajectory: AutoTrajectory = E_D.trajectory("E_rightStation")
    val stationToDTrajectory: AutoTrajectory = E_D.trajectory("rightStation_D")
    E_D.active().onTrue(
      Commands.sequence(

        reefETrajectory.resetOdometry(),

        // to reef
        Commands.deadline(reefETrajectory.cmd(), robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.LEFT),

        // to coral station
        robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
        reefEtoStationTrajectory.cmd().andThen(robot.drive.driveStop()),
        frc.team449.commands.Commands.Intake(robot),

        // to reef
        Commands.deadline(stationToDTrajectory.cmd(), robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.RIGHT),
      )
    )
    return E_D
  }

  fun reefEDhalf(): AutoRoutine {
    val E_DandHalf: AutoRoutine = autoFactory.newRoutine("L4 reef E,D and half")
    val reefETrajectory: AutoTrajectory = E_DandHalf.trajectory("right_E")
    val reefEtoStationTrajectory: AutoTrajectory = E_DandHalf.trajectory("E_rightStation")
    val stationToDTrajectory: AutoTrajectory = E_DandHalf.trajectory("rightStation_D")
    val reefDToStationTrajectory: AutoTrajectory = E_DandHalf.trajectory("D_rightStation")
    E_DandHalf.active().onTrue(
      Commands.sequence(

        reefETrajectory.resetOdometry(),

        // to reef
        Commands.deadline(reefETrajectory.cmd(), robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.LEFT),

        // to coral station
        robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
        reefEtoStationTrajectory.cmd().andThen(robot.drive.driveStop()),
        frc.team449.commands.Commands.Intake(robot),

        // to reef
        Commands.deadline(stationToDTrajectory.cmd(), robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.RIGHT),

        // to coral station
        robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
        reefDToStationTrajectory.cmd().andThen(robot.drive.driveStop()),
        frc.team449.commands.Commands.Intake(robot),
      )
    )
    return E_DandHalf
  }

  // l1 at reef J and then l4 at reef L
  fun reefJL(): AutoRoutine {
    val J_L: AutoRoutine = autoFactory.newRoutine("L1 reef J and L4 reef L")
    val reefJTrajectory: AutoTrajectory = J_L.trajectory("left_J")
    val reefJtoStationTrajectory: AutoTrajectory = J_L.trajectory("J_leftStation")
    val stationToLTrajectory: AutoTrajectory = J_L.trajectory("leftStation_L")
    J_L.active().onTrue(
      Commands.sequence(

        reefJTrajectory.resetOdometry(),

        // to reef
        Commands.deadline(reefJTrajectory.cmd(), robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.LEFT),

        // to coral station
        robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
        reefJtoStationTrajectory.cmd().andThen(robot.drive.driveStop()),
        frc.team449.commands.Commands.Intake(robot),

        // to reef
        Commands.deadline(stationToLTrajectory.cmd(), robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.RIGHT),
      )
    )
    return J_L
  }

  // l4 at reef J, K, L
  fun reefJKL(): AutoRoutine {
    val J_K_L: AutoRoutine = autoFactory.newRoutine("L4 reef J and L1 reef K")
    val reefJTrajectory: AutoTrajectory = J_K_L.trajectory("left_J")
    val reefJtoStationTrajectory: AutoTrajectory = J_K_L.trajectory("J_leftStation")
    val stationToKTrajectory: AutoTrajectory = J_K_L.trajectory("leftStation_K")
    val reefKtoStationTrajectory: AutoTrajectory = J_K_L.trajectory("K_leftStation")
    val stationToLTrajectory: AutoTrajectory = J_K_L.trajectory("leftStation_L")
    J_K_L.active().onTrue(
      Commands.sequence(

        reefJTrajectory.resetOdometry(),

        // to reef
        Commands.deadline(reefJTrajectory.cmd(), robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.RIGHT),

        // to coral station
        robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
        reefJtoStationTrajectory.cmd().andThen(robot.drive.driveStop()),
        frc.team449.commands.Commands.Intake(robot),

        // to reef
        Commands.deadline(stationToKTrajectory.cmd(), robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.LEFT),

        // to coral station
        robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
        reefKtoStationTrajectory.cmd().andThen(robot.drive.driveStop()),
        frc.team449.commands.Commands.Intake(robot),

        // to reef
        Commands.deadline(stationToLTrajectory.cmd(), robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.RIGHT),

      )
    )

    return J_K_L
  }

  // l4 at reef J, K, L
  fun reefEDC(): AutoRoutine {
    val E_D_C: AutoRoutine = autoFactory.newRoutine("L4 reef J and L1 reef K")
    val reefETrajectory: AutoTrajectory = E_D_C.trajectory("right_E")
    val reefEtoStationTrajectory: AutoTrajectory = E_D_C.trajectory("E_rightStation")
    val stationToDTrajectory: AutoTrajectory = E_D_C.trajectory("rightStation_D")
    val reefDtoStationTrajectory: AutoTrajectory = E_D_C.trajectory("D_rightStation")
    val stationToCTrajectory: AutoTrajectory = E_D_C.trajectory("rightStation_C")
    E_D_C.active().onTrue(
      Commands.sequence(

        reefETrajectory.resetOdometry(),

        // to reef
        Commands.deadline(reefETrajectory.cmd(), robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.RIGHT),

        // to coral station
        robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
        reefEtoStationTrajectory.cmd().andThen(robot.drive.driveStop()),
        frc.team449.commands.Commands.Intake(robot),

        // to reef
        Commands.deadline(stationToDTrajectory.cmd(), robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.LEFT),

        // to coral station
        robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
        reefDtoStationTrajectory.cmd().andThen(robot.drive.driveStop()),
        frc.team449.commands.Commands.Intake(robot),

        // to reef
        Commands.deadline(stationToCTrajectory.cmd(), robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        frc.team449.commands.Commands.ScoreL4(robot, FieldConstants.ReefSide.RIGHT),

      )
    )

    return E_D_C
  }

  // autoChooser that will be displayed on dashboard
  fun addOptions(autoChooser: AutoChooser) {
    autoChooser.addRoutine("RightTaxi", this::rightTaxi)
    autoChooser.addRoutine("LeftTaxi", this::leftTaxi)
    autoChooser.addRoutine("l1 G", this::l1reefG) // middle l1
    autoChooser.addRoutine("l4 E & l4 D ", this::reefED) // 2 piece l4
    autoChooser.addRoutine("l4 E & l4 D + half ", this::reefEDhalf) // 2 piece l4
    autoChooser.addRoutine("l4 J & l4 L", this::reefJL) // 2 piece l4
    autoChooser.addRoutine("l4 J,K,L", this::reefJKL) // 3 piece l4
    autoChooser.addRoutine("l4 E,D,C", this::reefEDC) // 3 piece l4
    autoChooser.addRoutine("The Goat", this::americanRoutine) // america
  }
}
