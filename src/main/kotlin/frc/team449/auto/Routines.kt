package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import choreo.trajectory.SwerveSample
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.Robot
import frc.team449.subsystems.superstructure.SuperstructureGoal
import kotlin.jvm.optionals.getOrDefault
import kotlin.math.PI
import edu.wpi.first.wpilibj.RobotBase
import frc.team449.commands.driveAlign.SimpleReefAlign
import java.util.Optional
import frc.team449.subsystems.FieldConstants

open class Routines(
  val robot: Robot
) {
  // Removed magic numbers
  private val xController: PIDController
    get() = PIDController(AutoConstants.DEFAULT_X_KP, 0.0, 0.0)
  private val yController: PIDController
    get() = PIDController(AutoConstants.DEFAULT_Y_KP, 0.0, 0.0)
  private val headingController: PIDController
    get() = PIDController(AutoConstants.DEFAULT_ROTATION_KP, 0.0, 0.0)

  init {
    headingController.enableContinuousInput(-Math.PI, Math.PI)
  }

  private fun followTrajectory(sample: SwerveSample) {
    val speeds = ChassisSpeeds(
      sample.vx + xController.calculate(robot.poseSubsystem.pose.x, sample.x),
      sample.vy + yController.calculate(robot.poseSubsystem.pose.y, sample.y),
      sample.omega + headingController.calculate(
        robot.poseSubsystem.pose.rotation.radians + if (DriverStation.getAlliance().getOrDefault(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) PI else 0.0, // Needs testing
        sample.heading
      )
    )
    val newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, robot.poseSubsystem.heading)
    // Apply the generated speeds
    robot.drive.set(newSpeeds)
  }

  val autoFactory = AutoFactory(
    robot.poseSubsystem::pose,
    robot.poseSubsystem::resetPoseChoreo,
    { sample: SwerveSample -> followTrajectory(sample) },
    true,
    robot.drive
  )

  // do nothing
  fun doNothing(): AutoRoutine {
    val nothing: AutoRoutine = autoFactory.newRoutine("Nothing")
    return nothing
  }

  fun americanRoutine(): AutoRoutine {
    val autoRoutine: AutoRoutine = autoFactory.newRoutine("L4 Routine")

    val l4fTrajectory: AutoTrajectory = autoRoutine.trajectory("Go To L4F")
    val rightStationTrajectory: AutoTrajectory = autoRoutine.trajectory("Go To Right Station(2)")
    val l1CTrajectory: AutoTrajectory = autoRoutine.trajectory("Go To L1C")
    val rightStationTrajectory2: AutoTrajectory = autoRoutine.trajectory("Go To Right Station(Again)")
    val l1bTrajectory: AutoTrajectory = autoRoutine.trajectory("Go To L1B")
    val leftStationTrajectory: AutoTrajectory = autoRoutine.trajectory("Go To Left Station")
    val l1kTrajectory: AutoTrajectory = autoRoutine.trajectory("Go To L4K")

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
        robot.drive.driveStop(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
          .alongWith(
            SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.RIGHT))
          )
          .andThen(robot.intake.outtakeCoral().until { !robot.intake.coralDetected() })
          .andThen(WaitCommand(0.15).onlyIf { RobotBase.isReal() })
          .andThen(rightStationTrajectory.cmd())
      )
    )

    rightStationTrajectory.done().onTrue(
      Commands.sequence(
        robot.drive.driveStop(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
          .andThen(robot.intake.intakeCoral().until { robot.intake.coralDetected() })
          .andThen(WaitCommand(0.15))
          .andThen(
            l1CTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE))
          )
      )
    )

    l1CTrajectory.done().onTrue(
      Commands.sequence(
        robot.drive.driveStop(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
          .alongWith(
            SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.RIGHT))
          )
          .andThen(robot.intake.outtakeCoral().until { !robot.intake.coralDetected() })
          .andThen(WaitCommand(0.15).onlyIf { RobotBase.isReal() })
          .andThen(rightStationTrajectory2.cmd())
      )
    )

    rightStationTrajectory2.done().onTrue(
      Commands.sequence(
        robot.drive.driveStop(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
          .andThen(robot.intake.intakeCoral().until { robot.intake.coralDetected() })
          .andThen(WaitCommand(0.15).onlyIf { RobotBase.isReal() })
          .andThen(
            l1bTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE))
          )
      )
    )

    l1bTrajectory.done().onTrue(
      Commands.sequence(
        robot.drive.driveStop(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
          .alongWith(
            SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.LEFT))
          )
          .andThen(robot.intake.outtakeCoral().until { !robot.intake.coralDetected() })
          .andThen(WaitCommand(0.15).onlyIf { RobotBase.isReal() })
          .andThen(leftStationTrajectory.cmd())
      )
    )

    leftStationTrajectory.done().onTrue(
      Commands.sequence(
        robot.drive.driveStop(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
          .andThen(robot.intake.intakeCoral().until { robot.intake.coralDetected() })
          .andThen(WaitCommand(0.15).onlyIf { RobotBase.isReal() })
          .andThen(
            l1kTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE))
          )
      )
    )

    l1kTrajectory.done().onTrue(
      Commands.sequence(
        robot.drive.driveStop(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
          .andThen(robot.intake.outtakeCoral().until { !robot.intake.coralDetected() })
          .andThen(WaitCommand(0.15)).andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW))
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

// l1 coral at reef E
  fun l1reefE(): AutoRoutine {
    val l1E: AutoRoutine = autoFactory.newRoutine("L1 reef E")
    val reefETrajectory: AutoTrajectory = l1E.trajectory("right_E")
    l1E.active().onTrue(
      Commands.sequence(
        reefETrajectory.resetOdometry(),
        Commands.parallel(
          Commands.sequence(reefETrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE)
        ),
        robot.intake.outtakeCoral()
      )
    )
    return l1E
  }

  // l4 coral at reef E
  fun l4reefE(): AutoRoutine {
    val l1E: AutoRoutine = autoFactory.newRoutine("L4 reef E")
    val reefETrajectory: AutoTrajectory = l1E.trajectory("right_E")
    l1E.active().onTrue(
      Commands.sequence(
        reefETrajectory.resetOdometry(),
        Commands.parallel(
          Commands.sequence(reefETrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)
        ),
        robot.intake.outtakeCoral()
      )
    )
    return l1E
  }

  // L1 coral at reef F
  fun l1reefF(): AutoRoutine {
    val F: AutoRoutine = autoFactory.newRoutine("L4 reef F")
    val reefFTrajectory: AutoTrajectory = F.trajectory("right_F")
    F.active().onTrue(
      Commands.sequence(
        reefFTrajectory.resetOdometry(),
        Commands.parallel(
          Commands.sequence(reefFTrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE)
        ),
        robot.intake.outtakeCoral()
      )
    )
    return F
  }

// L4 coral at reef G
  fun l4reefG(): AutoRoutine {
    val g: AutoRoutine = autoFactory.newRoutine("L4 reef G")
    val reefGTrajectory: AutoTrajectory = g.trajectory("middle_G")
    g.active().onTrue(
      Commands.sequence(
        reefGTrajectory.resetOdometry(),
        Commands.parallel(
          Commands.sequence(reefGTrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)
        ),
        robot.intake.outtakeCoral()
      )
    )
    return g
  }

  // l1 coral at reef J
  fun l1reefJ(): AutoRoutine {
    val J: AutoRoutine = autoFactory.newRoutine("L1 reef J")
    val reefJTrajectory: AutoTrajectory = J.trajectory("left_J")
    J.active().onTrue(
      Commands.sequence(
        reefJTrajectory.resetOdometry(),
        Commands.parallel(
          Commands.sequence(reefJTrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE)
        ),
        robot.intake.outtakeCoral()
      )
    )
    return J
  }

// l1 at reef E and then l4 at reef D
  fun reefED(): AutoRoutine {
    val E_D: AutoRoutine = autoFactory.newRoutine("L1 reef E and L4 reef D")
    val reefETrajectory: AutoTrajectory = E_D.trajectory("right_E")
    val reefEtoStationTrajectory: AutoTrajectory = E_D.trajectory("E_rightStation")
    val stationToDTrajectory: AutoTrajectory = E_D.trajectory("rightStation_D")
    E_D.active().onTrue(

      Commands.sequence(
        reefETrajectory.resetOdometry(),
        Commands.parallel(
          Commands.sequence(reefETrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE)
        ),
        robot.intake.outtakeCoral(),

        Commands.parallel(
          Commands.sequence(reefEtoStationTrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
          robot.intake.intakeCoral().andThen(WaitUntilCommand{robot.intake.coralDetected()})

        ),
        robot.intake.stop(),


        Commands.parallel(
          Commands.sequence(stationToDTrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)
        ),
        robot.intake.outtakeCoral(),
      )
    )
    return E_D
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
        Commands.parallel(
          Commands.sequence(reefJTrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE)
        ),
        robot.intake.outtakeCoral(),

        Commands.parallel(
          Commands.sequence(reefJtoStationTrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
          robot.intake.intakeCoral()
        ),
        robot.intake.stop(),

        Commands.parallel(
          Commands.sequence(stationToLTrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)
        ),
        robot.intake.outtakeCoral(),
      )
    )
    return J_L
  }

  // l1 at reef J and then l4 at reef K


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
        Commands.parallel(
          Commands.sequence(reefJTrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)
        ),
        robot.intake.outtakeCoral(),

        Commands.parallel(
          Commands.sequence(reefJtoStationTrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
          robot.intake.intakeCoral()
        ),
        robot.intake.stop(),

        Commands.parallel(
          Commands.sequence(stationToKTrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)
        ),
        robot.intake.outtakeCoral(),

        Commands.parallel(
          Commands.sequence(reefKtoStationTrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
          robot.intake.intakeCoral()
        ),
        robot.intake.stop(),

        Commands.parallel(
          Commands.sequence(stationToLTrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)
        ),
        robot.intake.outtakeCoral(),
      )
    )
    return J_K_L
  }

  // l4 at reef J, K, L
  fun reefJ_fullReefK(): AutoRoutine {
    val J_fullK: AutoRoutine = autoFactory.newRoutine("L4 reef J and L1 reef K")
    val reefJTrajectory: AutoTrajectory = J_fullK.trajectory("left_J")
    val reefJtoStationTrajectory: AutoTrajectory = J_fullK.trajectory("J_leftStation")
    val stationToKTrajectory: AutoTrajectory = J_fullK.trajectory("leftStation_K")
    val reefKtoStationTrajectory: AutoTrajectory = J_fullK.trajectory("K_leftStation")
    J_fullK.active().onTrue(

      Commands.sequence(
        reefJTrajectory.resetOdometry(),
        Commands.parallel(
          Commands.sequence(reefJTrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE)
        ),
        robot.intake.outtakeCoral(),

        Commands.parallel(
          Commands.sequence(reefJtoStationTrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
          robot.intake.intakeCoral()
        ),
        robot.intake.stop(),

        Commands.parallel(
          Commands.sequence(stationToKTrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L2_PREMOVE)
        ),
        robot.intake.outtakeCoral(),

        Commands.parallel(
          Commands.sequence(reefKtoStationTrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
          robot.intake.intakeCoral()
        ),
        robot.intake.stop(),

        Commands.parallel(
          Commands.sequence(stationToKTrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L3_PREMOVE)
        ),
        robot.intake.outtakeCoral(),

        Commands.parallel(
          Commands.sequence(reefKtoStationTrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
          robot.intake.intakeCoral()
        ),
        robot.intake.stop(),

        Commands.parallel(
          Commands.sequence(stationToKTrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)
        ),
        robot.intake.outtakeCoral()
      )
    )
    return J_fullK
  }

  // left side of the field spam l1
  fun leftSpamL1(): AutoRoutine {
    val spam: AutoRoutine = autoFactory.newRoutine(" spamming")
    val reefJTrajectory: AutoTrajectory = spam.trajectory("left_J")
    val reefJToLeftStation: AutoTrajectory = spam.trajectory("J_leftStation")
    val leftStationToReefK: AutoTrajectory = spam.trajectory("leftStation_K")
    val reefKToLeftStation: AutoTrajectory = spam.trajectory("K_leftStation")
    val leftStationReefL: AutoTrajectory = spam.trajectory("leftStation_L")
    val reefLtoLeftStation: AutoTrajectory = spam.trajectory("L_leftStation")
    spam.active().onTrue(
      Commands.sequence(

        reefJTrajectory.resetOdometry(),
        Commands.parallel(
          Commands.sequence(reefJTrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE)
        ),
        robot.intake.outtakeCoral(),

        Commands.parallel(
          Commands.sequence(reefJToLeftStation.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
          robot.intake.intakeCoral()
        ),
        robot.intake.stop(),

        Commands.parallel(
          Commands.sequence(leftStationToReefK.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE)
        ),
        robot.intake.outtakeCoral(),

        Commands.parallel(
          Commands.sequence(reefKToLeftStation.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
          robot.intake.intakeCoral()
        ),
        robot.intake.stop(),

        Commands.parallel(
          Commands.sequence(leftStationReefL.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE)
        ),
        robot.intake.outtakeCoral(),

        Commands.parallel(
          Commands.sequence(reefLtoLeftStation.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
          robot.intake.intakeCoral()
        ),
        robot.intake.stop(),

        Commands.parallel(
          Commands.sequence(leftStationToReefK.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE)
        ),
        robot.intake.outtakeCoral(),

        Commands.parallel(
          Commands.sequence(reefKToLeftStation.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
          robot.intake.intakeCoral()
        ),
        robot.intake.stop(),

        Commands.parallel(
          Commands.sequence(leftStationReefL.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE)
        ),
        robot.intake.outtakeCoral()

      )
    )
    return spam
  }

  // left side of the field spam l1
  fun rightSpamL1(): AutoRoutine {
    val spam: AutoRoutine = autoFactory.newRoutine(" spamming")
    val reefETrajectory: AutoTrajectory = spam.trajectory("right_E")
    val reefEToRightStation: AutoTrajectory = spam.trajectory("E_rightStation")
    val rightStationToReefD: AutoTrajectory = spam.trajectory("rightStation_D")
    val reefDToRightStation: AutoTrajectory = spam.trajectory("D_rightStation")
    val rightStationReefC: AutoTrajectory = spam.trajectory("rightStation_C")
    val reefCtoRightStation: AutoTrajectory = spam.trajectory("C_rightStation")
    spam.active().onTrue(
      Commands.sequence(

        reefETrajectory.resetOdometry(),
        Commands.parallel(
          Commands.sequence(reefETrajectory.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE)
        ),
        robot.intake.outtakeCoral(),

        Commands.parallel(
          Commands.sequence(reefEToRightStation.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
          robot.intake.intakeCoral()
        ),
        robot.intake.stop(),

        Commands.parallel(
          Commands.sequence(rightStationToReefD.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE)
        ),
        robot.intake.outtakeCoral(),

        Commands.parallel(
          Commands.sequence(reefDToRightStation.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
          robot.intake.intakeCoral()
        ),
        robot.intake.stop(),

        Commands.parallel(
          Commands.sequence(rightStationReefC.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE)
        ),
        robot.intake.outtakeCoral(),

        Commands.parallel(
          Commands.sequence(reefCtoRightStation.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
          robot.intake.intakeCoral()
        ),
        robot.intake.stop(),

        Commands.parallel(
          Commands.sequence(rightStationToReefD.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE)
        ),
        robot.intake.outtakeCoral(),

        Commands.parallel(
          Commands.sequence(reefDToRightStation.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
          robot.intake.intakeCoral()
        ),
        robot.intake.stop(),

        Commands.parallel(
          Commands.sequence(rightStationReefC.cmd(), robot.drive.driveStop()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE)
        ),
        robot.intake.outtakeCoral()

      )
    )
    return spam
  }

  // autoChooser that will be displayed on dashboard
  fun addOptions(autoChooser: AutoChooser) {
    autoChooser.addRoutine("Nothing", this::doNothing)
    autoChooser.addRoutine("RightTaxi", this::rightTaxi)
    autoChooser.addRoutine("LeftTaxi", this::leftTaxi)
    autoChooser.addRoutine("l4 E", this::l1reefE)
    autoChooser.addRoutine("l1 F", this::l1reefF)
    autoChooser.addRoutine("l1 J", this::l1reefJ)
    autoChooser.addRoutine("l1 E", this::l4reefE)
    autoChooser.addRoutine("l4 G", this::l4reefG)
    autoChooser.addRoutine("l1 E & l4 D ", this::reefED)
    autoChooser.addRoutine("l1 J & l4 L", this::reefJL)

    autoChooser.addRoutine("l4 J,K,L", this::reefJKL)
    autoChooser.addRoutine("l1 J + full K", this::reefJ_fullReefK)
    autoChooser.addRoutine("l1 rightSpam", this::rightSpamL1)
    autoChooser.addRoutine("l1 leftSpam", this::leftSpamL1)
    autoChooser.addRoutine("The Goat",this::americanRoutine)
  }
}
