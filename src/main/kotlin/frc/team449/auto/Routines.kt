package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import choreo.trajectory.SwerveSample
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.*
import frc.team449.Robot
import frc.team449.commands.Commands.ScoreL4
import frc.team449.commands.driveAlign.SimpleReefAlign
import frc.team449.subsystems.FieldConstants
import frc.team449.subsystems.superstructure.SuperstructureGoal
import java.util.Optional

open class Routines(
  val robot: Robot
) {

  private val autoFactory = AutoFactory(
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

  /** link to starting position on the field: https://docs.google.com/document/d/1SOzIJDgJ0GRSVnNTcBhaFfltvHw0IjJTEUsAZbI2hW4/edit?usp=sharing  **/
  /** left and right are from the driver's pov **/

  fun taxi(): AutoRoutine {
    val rTaxi: AutoRoutine = autoFactory.newRoutine("Right Taxi")
    val rTaxiTrajectory: AutoTrajectory = rTaxi.trajectory("prev/taxiRight")
    rTaxi.active().onTrue(Commands.sequence(rTaxiTrajectory.resetOdometry(), rTaxiTrajectory.cmd(), robot.drive.driveStop()))
    return rTaxi
  }

  fun middleRoutine(): AutoRoutine {
    val middleRoutine = autoFactory.newRoutine("one l4 ")
    val forward = middleRoutine.trajectory("OneL4/1")
    val end = middleRoutine.trajectory("OneL4/2")

    middleRoutine.active().onTrue(
      Commands.sequence(
        robot.intake.resetPiece(),
        forward.resetOdometry(),
        forward.cmd().alongWith(
          WaitCommand(0.5).andThen(
            robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)
          )
        )
      )
    )

    forward.done().onTrue(
      Commands.sequence(
        ScoreL4(robot, FieldConstants.ReefSide.LEFT),
        end.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)),
        robot.drive.driveStop()
      )

    )
    return middleRoutine
  }

  /**Ground Intake Autos**/

  private fun getScoreCommand(reefLevel: Int): (FieldConstants.ReefSide) -> Command {
    return when (reefLevel) {
      2 -> { side: FieldConstants.ReefSide -> scoreL2PivotDirectional(side) }
      4 -> { side: FieldConstants.ReefSide -> scoreL4PivotSideDirectional(side) }
      else -> { side: FieldConstants.ReefSide -> scoreL4PivotSideDirectional(side) }
    }
  }

  // back l4 and then sides 2 l4
  private fun threeL4(direction: String): AutoRoutine {
    val middlesides = autoFactory.newRoutine("3 l4 ${if (direction == "r") "Right" else "Left"}")
    val preloadScore = middlesides.trajectory("middleSides/1$direction")
    val firstPickup = middlesides.trajectory("middleSides/2$direction")
    val firstPresagedScore = middlesides.trajectory("middleSides/3$direction")
    val secondPickup = middlesides.trajectory("middleSides/4$direction")
    val secondPresagedScore = middlesides.trajectory("middleSides/5$direction")
    val end = middlesides.trajectory("middleSides/end$direction")

    middlesides.active().onTrue(
      Commands.sequence(
        robot.intake.resetPiece(),
        preloadScore.resetOdometry().alongWith(robot.intake.stopMotors()),
        preloadScore.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOT)
            .withDeadline(WaitCommand(1.5))
        )
      )
    )

    preloadScore.done().onTrue(
      Commands.sequence(
        scoreL4PivotSideDirectional(if (direction == "l") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        firstPickup.cmd().alongWith(intakeCoral()),
        robot.drive.driveStop(),
        firstPresagedScore.cmd().alongWith(
          WaitCommand(0.52).andThen(
            robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOT)
          )
        )
      )
    )

    firstPresagedScore.done().onTrue(
      Commands.sequence(
        scoreL4PivotSideDirectional(if (direction == "r") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        secondPickup.cmd().alongWith(intakeCoral()),
        robot.drive.driveStop(),
        secondPresagedScore.cmd().alongWith(
          WaitCommand(0.52).andThen(
            robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOT)
          )
        )
      )
    )
    secondPresagedScore.done().onTrue(
      Commands.sequence(
        scoreL4PivotSideDirectional(if (direction == "l") FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        end.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW))
      )
    )

    return middlesides
  }

  fun left3L4(): AutoRoutine {
    return threeL4("l")
  }

  fun right3L4(): AutoRoutine {
    return threeL4("r")
  }

  private fun groundBack2L4L2(direction: String, reefLevels: IntArray): AutoRoutine {
    val routine = autoFactory.newRoutine("2 l4 and l2 ${if (direction == "r") "Right" else "Left"}")
    val scorePreloadB = routine.trajectory("TwoL4L2/1$direction")
    val pickupMiddle = routine.trajectory("TwoL4L2/2$direction")
    val scoreMiddleA = routine.trajectory("TwoL4L2/3$direction")
    val pickupLeft = routine.trajectory("TwoL4L2/4$direction")
    val scoreRightB = routine.trajectory("TwoL4L2/5$direction")
    val pickupRight = routine.trajectory("TwoL4L2/6$direction")
    val scoreLeftA = routine.trajectory("TwoL4L2/7$direction")

    val firstPickupTime = 3.1 // same on both
    val secondPickupTime = 2.8 // same on both
    val thirdPickupTime = 3.0

    val missMidPickup = routine.trajectory("TwoL4L2/failmid1$direction")
    val missMidScore = routine.trajectory("TwoL4L2/failmid2$direction")
    val missMidSecondPickup = routine.trajectory("TwoL4L2/failmid3$direction")
    val missMidSecondScore = routine.trajectory("TwoL4L2/failmid4$direction") // second score impossible on miss

    val missFarPickup = routine.trajectory("TwoL4L2/failfar1$direction")
    val missFarScore = routine.trajectory("TwoL4L2/failfar2$direction")

    // first path
    routine.active().onTrue(
      Commands.sequence(
        robot.intake.resetPiece(),
        scorePreloadB.resetOdometry(),
        scorePreloadB.cmd().alongWith(getPremoveCommand(reefLevels[0], 1.65))
      )
    )

    // first piece score second piece pickup
    scorePreloadB.done().onTrue(
      Commands.sequence(

        robot.drive.driveStop(),
        scoreCoral(),
        pickupMiddle.cmd()
          .alongWith(
            robot.intake.intakeToVertical()).alongWith(
              robot.superstructureManager.requestGoal(SuperstructureGoal.GROUND_INTAKE_CORAL)
            )
          .withTimeout(firstPickupTime + AutoConstants.INTAKE_TIMEOUT),

        ConditionalCommand(
          // if we picked it up regularly
          scoreMiddleA.cmd()
            .alongWith(getPremoveCommand(reefLevels[1], 1.15)),

          // if we didnt
          Commands.sequence(
            // do the miss pickup
            missMidPickup.cmd()
              .alongWith(
                // if its partially intakken outtake it
                robot.intake.outtakeL1()
                  .withTimeout(0.5)
                  .andThen(robot.intake.intakeToVertical()).alongWith(
                    robot.superstructureManager.requestGoal(SuperstructureGoal.GROUND_INTAKE_CORAL)
                  )
              )
              .andThen(robot.drive.driveStop())
              .withTimeout(firstPickupTime + AutoConstants.INTAKE_TIMEOUT),

            ConditionalCommand(
              // if we landed the miss pickup
              missMidScore.cmd()
                .alongWith(getPremoveCommand(reefLevels[1], 0.85)),

              // if we did not we're buns but anyways
              Commands.sequence(
                // pick up the third
                robot.drive.driveStop(),
                missFarPickup.cmd()
                  .alongWith(
                    robot.intake.outtakeL1()
                      .withTimeout(0.5)
                      .andThen(robot.intake.intakeToVertical()).alongWith(
                        robot.superstructureManager.requestGoal(SuperstructureGoal.GROUND_INTAKE_CORAL)
                      )
                  )
                  .andThen(robot.drive.driveStop()),

                // score it
                missFarScore.cmd().alongWith(
                  getPremoveCommand(reefLevels[1], 1.0)
                ),
                robot.drive.driveStop(),
                scoreCoral(),
                robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
                // end the jaunt cause we running out of time lmao

              )

            ) { robot.intake.coralDetected() || !RobotBase.isReal() }
          )

        ) { robot.intake.coralDetected() || !RobotBase.isReal() }
      )
    )

    // first miss backup second score logic
    missMidScore.done().onTrue(
      Commands.sequence(

        robot.drive.driveStop(),
        scoreCoral(),

        missMidSecondPickup.cmd()
          .alongWith(robot.intake.intakeToVertical()).alongWith(
            robot.superstructureManager.requestGoal(SuperstructureGoal.GROUND_INTAKE_CORAL)
          )
          .andThen(robot.drive.driveStop()),

        missMidSecondScore.cmd()
          .alongWith(getPremoveCommand(reefLevels[2], 1.15)),
        robot.drive.driveStop(),

        scoreCoral(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
      )
    )

    // second piece score third piece pickup
    scoreMiddleA.done().onTrue(

      Commands.sequence(

        robot.drive.driveStop(),
        scoreCoral(),

        pickupLeft.cmd().alongWith(
        (robot.superstructureManager.requestGoal(SuperstructureGoal.GROUND_INTAKE_CORAL))).alongWith(
        (robot.intake.intakeToVertical()))
          .andThen(robot.drive.driveStop())
          .withTimeout(secondPickupTime + AutoConstants.INTAKE_TIMEOUT),

        ConditionalCommand(
          // picked it up reg
          scoreRightB.cmd()
            .alongWith(getPremoveCommand(reefLevels[2], 1.15)),
          // safety
          Commands.sequence(
            missFarPickup.cmd()
              .alongWith(
                robot.intake.outtakeL1()
                  .withTimeout(0.5)
                  .andThen(robot.intake.intakeToVertical()).alongWith(
                    robot.superstructureManager.requestGoal(SuperstructureGoal.GROUND_INTAKE_CORAL)
                  )
              )
              .andThen(robot.drive.driveStop()),

            missFarScore.cmd().alongWith(
              getPremoveCommand(reefLevels[2], 0.85)
            ),
            robot.drive.driveStop(),
            scoreCoral(),
            robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
          )

        ) { robot.intake.coralDetected() || !RobotBase.isReal() }
      )

    )

    // third piece score
    scoreRightB.done()
      .onTrue(
        Commands.sequence(
          robot.drive.driveStop(),
          scoreCoral(),
          robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
        )
      )

    return routine
  }

  fun rightGroundBack2L4L2(): AutoRoutine {
    return groundBack2L4L2("r", intArrayOf(2, 4, 4, 2))
  }

  fun leftGroundBack2L4L2(): AutoRoutine {
    return groundBack2L4L2("l", intArrayOf(2, 4, 4, 2))
  }

  private fun severnAuto(): AutoRoutine {
    val routine = autoFactory.newRoutine("severn auto")
    val waitTimes = listOf(1.0, 1.5, 1.4)

    val scoreFirstPiece = routine.trajectory("subby3L4/1")
    val pickupSecondPiece = routine.trajectory("subby3L4/2")
    val scoreSecondPiece = routine.trajectory("subby3L4/3")
    val pickupThirdPiece = routine.trajectory("subby3L4/4")
    val scoreThirdPiece = routine.trajectory("subby3L4/5")

    routine.active().onTrue(
      Commands.sequence(
        robot.intake.resetPiece(),
        scoreFirstPiece.resetOdometry(),
        scoreFirstPiece.cmd().alongWith(getPremoveCommand(4, waitTimes[0]))
      )
    )

    scoreFirstPiece.done().onTrue(
      Commands.sequence(
        robot.drive.driveStop(),
        scoreCoral(),
        pickupSecondPiece.cmd()
          .alongWith(WaitCommand(1.5)
            .andThen(robot.intake.intakeToVertical()))
          .alongWith(
            robot.superstructureManager.requestGoal(SuperstructureGoal.STATION_INTAKE)
          )
      )
    )

    pickupSecondPiece.done().onTrue(
      Commands.sequence(
        robot.drive.driveStop(),
        WaitUntilCommand { robot.intake.hasPiece() },
        scoreSecondPiece.cmd().alongWith(getPremoveCommand(4, waitTimes[1]))
      )
    )

    scoreSecondPiece.done().onTrue(
      Commands.sequence(
        robot.drive.driveStop(),
        scoreCoral(),
        pickupThirdPiece.cmd()
          .alongWith(WaitCommand(1.5)
            .andThen(robot.intake.intakeToVertical()))
          .alongWith(
            robot.superstructureManager.requestGoal(SuperstructureGoal.STATION_INTAKE)
          )
      )
    )

    pickupThirdPiece.done().onTrue(
      Commands.sequence(
        robot.drive.driveStop(),
        WaitUntilCommand { robot.intake.hasPiece() },
        scoreThirdPiece.cmd().alongWith(getPremoveCommand(4, waitTimes[2]))
      )
    )

    return routine
  }

  private fun algaeAuto(threePointFive: Boolean = false, closeLaunch: Boolean = true): AutoRoutine {
    val routine = autoFactory.newRoutine("algae auto")
    val algaeAutoType = if (closeLaunch) "OneL4ThreeAlClose" else "OneL4ThreeALFar"
    val farWaitTimes = listOf(0.8, 0.9, 1.2, 1.2)
    val closeWaitTimes = listOf(0.8, 0.83, 1.13, 0.93)
    val waitTimes = closeWaitTimes
    val preloadedL4Score = routine.trajectory("$algaeAutoType/1")
    val algaePickupMiddle = routine.trajectory("$algaeAutoType/2")
    val netScoreMiddle = routine.trajectory("$algaeAutoType/3")
    val algaePickupLeft = routine.trajectory("$algaeAutoType/4")
    val netScoreLeft = routine.trajectory("$algaeAutoType/5")
    val algaePickupRight = routine.trajectory("$algaeAutoType/6")
    val netScoreRight = routine.trajectory("${if (threePointFive) "AlgaeSafety" else algaeAutoType}/7")
    val getTaxi = routine.trajectory("${if (threePointFive) "AlgaeSafety" else algaeAutoType}/8")

    routine.active().onTrue(
      Commands.sequence(
        robot.intake.resetPiece(),
        preloadedL4Score.resetOdometry(),
        preloadedL4Score.cmd().alongWith(getPremoveCommand(-4, waitTimes[0]))
      )
    )

    preloadedL4Score.done().onTrue(
      Commands.sequence(
        robot.drive.driveStop(),
        scoreCoral(false),
        algaePickupMiddle.cmd().andThen(robot.drive.driveStop())
          .alongWith(intakeAlgae(2)),
        netScoreMiddle.cmd()
          .alongWith(getPremoveCommand(5, waitTimes[1])),
      )
    )

    netScoreMiddle.done().onTrue(
      Commands.sequence(
        robot.drive.driveStop(),
        scoreAlgaePivot(),
        algaePickupLeft.cmd().andThen(robot.drive.driveStop())
          .alongWith(intakeAlgae(3))
          .until { robot.intake.algaeDetected() },
        netScoreLeft.cmd()
          .alongWith(getPremoveCommand(5, waitTimes[2])),
      )
    )

    netScoreLeft.done().onTrue(
      ConditionalCommand(
        Commands.sequence(
          robot.drive.driveStop(),
          scoreAlgaePivot(),
          algaePickupRight.cmd().andThen(robot.drive.driveStop())
            .alongWith(intakeAlgae(3))
            .until { robot.intake.algaeDetected() },
          netScoreRight.cmd()
            .alongWith(getPremoveCommand(5, waitTimes[3])),
        ),
        Commands.sequence(
          robot.drive.driveStop(),
          scoreAlgaePivot(),
          algaePickupRight.cmd().andThen(robot.drive.driveStop())
            .alongWith(intakeAlgae(3))
            .until { robot.intake.algaeDetected() },
          netScoreRight.cmd(),
        )
      ) { !threePointFive }
    )

    netScoreRight.done().onTrue(
      ConditionalCommand(
        Commands.sequence(
          robot.drive.driveStop(),
          scoreAlgaePivot(),
          robot.superstructureManager.requestGoal(
            SuperstructureGoal.SuperstructureState(
              SuperstructureGoal.NET_PIVOT.pivot,
              SuperstructureGoal.STOW.elevator,
              SuperstructureGoal.NET_PIVOT.wrist,
              SuperstructureGoal.NET_PIVOT.driveDynamics
            )
          ).alongWith(WaitCommand(0.25).andThen(getTaxi.cmd()))
        ),
        InstantCommand()
      ) { !threePointFive }

    )

    return routine
  }

  private fun hateKotlin(): AutoRoutine { return algaeAuto(true) }

  // autoChooser that will be displayed on dashboard
  fun addOptions(autoChooser: AutoChooser) {
    autoChooser.addRoutine("lollipop right", this::rightGroundBack2L4L2)
    autoChooser.addRoutine("lollipop Left", this::leftGroundBack2L4L2)

    autoChooser.addRoutine("Center L4 & 3 Algae", this::algaeAuto)
    autoChooser.addRoutine("Center L4 & 2.5 Algae", this::hateKotlin)

    autoChooser.addRoutine("Severn Left", this::severnAuto)

    autoChooser.addRoutine("Center 1 L4", this::middleRoutine)

    autoChooser.addRoutine("Taxi", this::taxi)

    autoChooser.addRoutine("Center 1 L4", this::middleRoutine)
  }

  private fun scoreL4PivotSideDirectional(reefSide: FieldConstants.ReefSide): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PIVOT)
      .alongWith(
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(reefSide), translationSpeedLim = 1.0, translationAccelLim = 1.95)
          .andThen(PrintCommand("Actually reached auto tolerance!"))
          .withTimeout(2.0)
      )
      .andThen(WaitCommand(0.10))
      .andThen(robot.intake.outtakeCoralPivot())
      .andThen(PrintCommand("Outook Piece!"))
  }

  private fun scoreCoral(pivotSide: Boolean = true): Command {
    return WaitUntilCommand { robot.pivot.atSetpoint() && robot.elevator.atSetpoint() && robot.wrist.atSetpoint() }
      .andThen(WaitCommand(0.15))
      .andThen(
        ConditionalCommand(
          robot.intake.outtakeCoralPivot(),
          robot.intake.outtakeCoral()
        ) { pivotSide }
      )
      .andThen(PrintCommand("Outook Piece!"))
  }

  private fun scoreAlgaePivot(): Command {
    return WaitUntilCommand { robot.pivot.atSetpoint() && robot.elevator.atSetpoint() && robot.wrist.atSetpoint() }
      .andThen(robot.intake.outtakeAlgae())
      .andThen(PrintCommand("Outook Piece!"))
  }

  private fun scoreL2PivotDirectional(reefSide: FieldConstants.ReefSide): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.L2_PIVOT)
      .alongWith(
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(reefSide), translationSpeedLim = 1.0, translationAccelLim = 1.95)
          .andThen(PrintCommand("Actually reached auto tolerance!"))
          .withTimeout(2.0)
      )
      .andThen(WaitCommand(0.10))
      .andThen(robot.intake.outtakeCoralPivot())
      .andThen(PrintCommand("Outook Piece!"))
  }

  private fun intakeCoral(): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.GROUND_INTAKE_CORAL)
      .alongWith(robot.intake.intakeToVertical())
  }

  private fun intakeAlgae(level: Int): Command {
    return ConditionalCommand(
      robot.superstructureManager.requestGoal(SuperstructureGoal.L2_ALGAE_INTAKE),
      robot.superstructureManager.requestGoal(
        SuperstructureGoal.SuperstructureState(
          SuperstructureGoal.NET_PIVOT.pivot,
          SuperstructureGoal.L3_ALGAE_INTAKE.elevator,
          SuperstructureGoal.NET_PIVOT.wrist,
          SuperstructureGoal.L3_ALGAE_INTAKE.driveDynamics
        )
      ).andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.L3_ALGAE_INTAKE))
    ) { level == 2 }
      .alongWith(WaitCommand(0.3).andThen(robot.intake.intakeAlgae()))
  }

  private fun getPremoveCommand(reefLevel: Int, waitTime: Double = 0.0): Command {
    return when (reefLevel) {
      2 -> robot.superstructureManager.requestGoal(SuperstructureGoal.L2_PIVOT).alongWith(robot.intake.moveCoralPivotSide())
      4 -> Commands.sequence(
        robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOT).alongWith(robot.intake.moveCoralPivotSide())
          .withDeadline(WaitCommand(waitTime)),
        robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PIVOT)
      )
      -4 -> Commands.sequence(
        robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE).alongWith(robot.intake.moveCoralOppSide())
          .withDeadline(WaitCommand(waitTime)),
        robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
      )
      5 -> Commands.deadline( // 5 is net
        Commands.sequence(
          robot.superstructureManager.requestGoal(SuperstructureGoal.NET_PREMOVE_PIVOT)
            .withDeadline(WaitCommand(waitTime)),
          robot.superstructureManager.requestGoal(SuperstructureGoal.NET_PIVOT)
        ),
      )
      else -> InstantCommand()
    }
  }
}
