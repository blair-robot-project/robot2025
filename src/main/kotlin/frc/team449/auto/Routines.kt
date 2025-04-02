package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import choreo.trajectory.SwerveSample
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.*
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

  /** link to starting position on the field: https://docs.google.com/document/d/1SOzIJDgJ0GRSVnNTcBhaFfltvHw0IjJTEUsAZbI2hW4/edit?usp=sharing  **/
  /** left and right are from the driver's pov **/

  fun taxi(): AutoRoutine {
    val rTaxi: AutoRoutine = autoFactory.newRoutine("Right Taxi")
    val rTaxiTrajectory: AutoTrajectory = rTaxi.trajectory("prev/taxiRight")
    rTaxi.active().onTrue(Commands.sequence(rTaxiTrajectory.resetOdometry(), rTaxiTrajectory.cmd(), robot.drive.driveStop()))
    return rTaxi
  }
  fun middleRoutine(): AutoRoutine {
    val middleRoutine = autoFactory.newRoutine("prev/middle Test")
    val test = middleRoutine.trajectory("prev/middle test")

    middleRoutine.active().onTrue(
      Commands.sequence(
        test.resetOdometry(),
        test.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE))
      )
    )

    test.done().onTrue(
      Commands.sequence(
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.LEFT), translationSpeedLim = 2.0, translationAccelLim = 1.4).alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
        ),
        robot.drive.driveStop(),
        WaitCommand(0.15),
        robot.intake.outtakeCoral().andThen(WaitUntilCommand { !robot.intake.coralDetected() }),
        WaitCommand(0.15).onlyIf { RobotBase.isReal() },
        robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
      )
    )

    return middleRoutine
  }
  fun americanRoutine(): AutoRoutine {
    val autoRoutine = autoFactory.newRoutine("L4 Routine")

    val l4ETrajectory = autoRoutine.trajectory("ThreeL4Right/1")
    val rightStationTrajectory = autoRoutine.trajectory("ThreeL4Right/2")
    val l4DTrajectory = autoRoutine.trajectory("ThreeL4Right/3")
    val rightStationTrajectory2 = autoRoutine.trajectory("ThreeL4Right/4")
    val l4CTrajectory = autoRoutine.trajectory("ThreeL4Right/5")

    autoRoutine.active().onTrue(
      Commands.sequence(
        l4ETrajectory.resetOdometry(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)
          .alongWith(
            robot.intake.stop(),
            l4ETrajectory.cmd()
          )
      ),
    )

    l4ETrajectory.done().onTrue(
      ScoreL4PivotSide(robot, FieldConstants.ReefSide.LEFT)
        .andThen(
          rightStationTrajectory.cmd()
            .alongWith(PremoveIntake(robot))
        )
    )

    rightStationTrajectory.done().onTrue(
      Commands.sequence(
        Intake(robot),
        l4DTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE).beforeStarting(WaitCommand(0.5)))
      )
    )

    l4DTrajectory.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.RIGHT),
        PremoveIntake(robot).alongWith(
          rightStationTrajectory2.cmd()
        )
      )
    )

    rightStationTrajectory2.done().onTrue(
      Commands.sequence(
        Intake(robot),
        l4CTrajectory.cmd().alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE).beforeStarting(
            WaitCommand(0.5)
          )
        )
      )
    )

    l4CTrajectory.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.LEFT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
      )
    )

    return autoRoutine
  }
  fun LeftamericanRoutine(): AutoRoutine {
    val leftAutoRoutine = autoFactory.newRoutine("Left L4 Routine")

    val l4jTrajectory = leftAutoRoutine.trajectory("ThreeL4Left/1")
    val lefStationTrajectory = leftAutoRoutine.trajectory("ThreeL4Left/2")
    val l4kTrajectory = leftAutoRoutine.trajectory("ThreeL4Left/3")
    val leftStationTraj2 = leftAutoRoutine.trajectory("ThreeL4Left/4")
    val l4LTrajectory = leftAutoRoutine.trajectory("ThreeL4Left/5")

    leftAutoRoutine.active().onTrue(
      Commands.sequence(
        l4jTrajectory.resetOdometry(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE).alongWith(
          robot.intake.stop(),
          l4jTrajectory.cmd()
        ),
        PrintCommand("Traveling to L4")
      )
    )

    l4jTrajectory.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.RIGHT),
        lefStationTrajectory.cmd().alongWith(PremoveIntake(robot))
      )
    )

    lefStationTrajectory.done().onTrue(
      Commands.sequence(
        Intake(robot),
        l4kTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE).beforeStarting(WaitCommand(0.5)))
      )
    )

    l4kTrajectory.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.LEFT),
        PremoveIntake(robot).alongWith(
          leftStationTraj2.cmd()
        )
      )
    )

    leftStationTraj2.done().onTrue(
      Commands.sequence(
        Intake(robot),
        l4LTrajectory.cmd()
          .alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE).beforeStarting(WaitCommand(0.5)))
      )
    )

    l4LTrajectory.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.RIGHT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
      )
    )
    return leftAutoRoutine
  }

  /**Ground Intake Autos**/

/******
   uncomment all the
"//.onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }"
while testing on a real robot
   *****/

  // three l4 starting from a side then the back two reefs then half
  fun rightGround3L4Half(): AutoRoutine {
    val ground3halfRight = autoFactory.newRoutine("3 l4 and half")
    val preloadScore = ground3halfRight.trajectory("GroundThreeHalf/1r")
    val firstPickup = ground3halfRight.trajectory("GroundThreeHalf/2r")
    val firstPresagedScore = ground3halfRight.trajectory("GroundThreeHalf/3r")
    val secondPickup = ground3halfRight.trajectory("GroundThreeHalf/4r")
    val secondPresagedScore = ground3halfRight.trajectory("GroundThreeHalf/5r")
    val thirdPickup = ground3halfRight.trajectory("GroundThreeHalf/6r")
    val thirdPresagedScore = ground3halfRight.trajectory("GroundThreeHalf/7r")

    ground3halfRight.active().onTrue(
      Commands.sequence(
        preloadScore.resetOdometry().alongWith(robot.intake.stop()),
        preloadScore.cmd().alongWith(
          WaitCommand(1.5),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
        )
      )
    )

    preloadScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.LEFT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
        firstPickup.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),
        (
          firstPresagedScore.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )
          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )

    firstPresagedScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.RIGHT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
        secondPickup.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),
        (
          secondPresagedScore.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )
          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )

    secondPresagedScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.LEFT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
        GroundIntake(robot).alongWith(
          thirdPickup.cmd()
        )
      )
    )

    thirdPickup.done().onTrue(
      robot.drive.driveStop()
    )

    return ground3halfRight
  }
  fun leftGround3L4Half(): AutoRoutine {
    val ground3halfLeft = autoFactory.newRoutine("3 l4 and half")
    val preloadScore = ground3halfLeft.trajectory("GroundThreeHalf/1l")
    val firstPickup = ground3halfLeft.trajectory("GroundThreeHalf/2l")
    val firstPresagedScore = ground3halfLeft.trajectory("GroundThreeHalf/3l")
    val secondPickup = ground3halfLeft.trajectory("GroundThreeHalf/4l")
    val secondPresagedScore = ground3halfLeft.trajectory("GroundThreeHalf/5l")
    val thirdPickup = ground3halfLeft.trajectory("GroundThreeHalf/6l")

    ground3halfLeft.active().onTrue(
      Commands.sequence(
        preloadScore.resetOdometry().alongWith(robot.intake.stop()),
        preloadScore.cmd().alongWith(
          WaitCommand(1.5),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
        )
      )
    )

    preloadScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.RIGHT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
        firstPickup.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),
        (
          firstPresagedScore.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )
          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )

    firstPresagedScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.LEFT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
        secondPickup.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),
        (
          secondPresagedScore.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )
          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )

    secondPresagedScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.RIGHT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
        GroundIntake(robot).alongWith(
          thirdPickup.cmd()
        )
      )
    )

    thirdPickup.done().onTrue(
      robot.drive.driveStop()
    )

    return ground3halfLeft
  }

  // back l4 and then sides 2 l4
  fun left3L4(): AutoRoutine {
    val middlesides = autoFactory.newRoutine("3 l4")
    val preloadScore = middlesides.trajectory("middleSides/1l")
    val firstPickup = middlesides.trajectory("middleSides/2l")
    val firstPresagedScore = middlesides.trajectory("middleSides/3l")
    val secondPickup = middlesides.trajectory("middleSides/4l")
    val secondPresagedScore = middlesides.trajectory("middleSides/5l")

    middlesides.active().onTrue(
      Commands.sequence(
        preloadScore.resetOdometry().alongWith(robot.intake.stop()),
        preloadScore.cmd().alongWith(
          WaitCommand(1.5),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
        )
      )
    )

    preloadScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.LEFT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
        firstPickup.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),
        (
          firstPresagedScore.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )
          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )

    firstPresagedScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.RIGHT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
        secondPickup.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),
        (
          secondPresagedScore.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )
          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )
    secondPresagedScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.LEFT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
      )
    )

    return middlesides
  }
  fun right3L4(): AutoRoutine {
    val middlesides = autoFactory.newRoutine("3 l4")
    val preloadScore = middlesides.trajectory("middleSides/1r")
    val firstPickup = middlesides.trajectory("middleSides/2r")
    val firstPresagedScore = middlesides.trajectory("middleSides/3r")
    val secondPickup = middlesides.trajectory("middleSides/4r")
    val secondPresagedScore = middlesides.trajectory("middleSides/5r")

    middlesides.active().onTrue(
      Commands.sequence(
        preloadScore.resetOdometry().alongWith(robot.intake.stop()),
        preloadScore.cmd().alongWith(
          WaitCommand(1.5),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
        )
      )
    )

    preloadScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.RIGHT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
        firstPickup.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),
        (
          firstPresagedScore.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )
          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )

    firstPresagedScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.LEFT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
        secondPickup.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),
        (
          secondPresagedScore.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )
          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )
    secondPresagedScore.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.RIGHT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
      )
    )

    return middlesides
  }

  // two l4 and two l2 on the back reef branches
  fun rightGroundBack2L4l2(): AutoRoutine {
    val rightBack2l4l2 = autoFactory.newRoutine("2 l4 and l2")
    val scorePreloadB = rightBack2l4l2.trajectory("TwoL4L2/1r")
    val pickupMiddle = rightBack2l4l2.trajectory("TwoL4L2/2r")
    val scoreMiddleA = rightBack2l4l2.trajectory("TwoL4L2/3r")
    val pickupLeft = rightBack2l4l2.trajectory("TwoL4L2/4r")
    val scoreRightB = rightBack2l4l2.trajectory("TwoL4L2/5r")
    val pickupRight = rightBack2l4l2.trajectory("TwoL4L2/6r")
    val scoreLeftA = rightBack2l4l2.trajectory("TwoL4L2/7r")

    rightBack2l4l2.active().onTrue(
      Commands.sequence(
        scorePreloadB.resetOdometry().alongWith(
          robot.intake.stop()
        ),
        scorePreloadB.cmd().alongWith(
          WaitCommand(2.0),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
        )
      )
    )

    scorePreloadB.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.LEFT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
        pickupMiddle.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),
        (
          scoreMiddleA.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )

          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )

    scoreMiddleA.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.RIGHT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
        pickupLeft.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),

        (
          scoreRightB.cmd().alongWith(
            WaitCommand(0.74).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )
          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )

    scoreRightB.done()
      .onTrue(
        Commands.sequence(
          ScoreL2PivotSide(robot, FieldConstants.ReefSide.RIGHT),
          robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
          pickupRight.cmd().alongWith(GroundIntake(robot)),
          robot.drive.driveStop(),
          (
            scoreLeftA.cmd().alongWith(
              WaitCommand(0.68).andThen(
                robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
              )
            )
            ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
        )
      )

    scoreLeftA.done()
      .onTrue(
        Commands.sequence(
          ScoreL2PivotSide(robot, FieldConstants.ReefSide.LEFT),
          robot.superstructureManager.requestGoal(SuperstructureGoal.STOW),
          robot.drive.driveStop(),

        )
      )

    return rightBack2l4l2
  }
  fun leftGroundBack2L4l2(): AutoRoutine {
    val leftBack2l4l2 = autoFactory.newRoutine("2 l4 and l2")
    val scorePreloadB = leftBack2l4l2.trajectory("TwoL4L2/1l")
    val pickupMiddle = leftBack2l4l2.trajectory("TwoL4L2/2l")
    val scoreMiddleA = leftBack2l4l2.trajectory("TwoL4L2/3l")
    val pickupLeft = leftBack2l4l2.trajectory("TwoL4L2/4l")
    val scoreRightB = leftBack2l4l2.trajectory("TwoL4L2/5l")
    val pickupRight = leftBack2l4l2.trajectory("TwoL4L2/6l")
    val scoreLeftA = leftBack2l4l2.trajectory("TwoL4L2/7l")

    leftBack2l4l2.active().onTrue(
      Commands.sequence(
        scorePreloadB.resetOdometry().alongWith(
          robot.intake.stop()
        ),
        scorePreloadB.cmd().alongWith(
          WaitCommand(2.0),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
        )
      )
    )

    scorePreloadB.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.LEFT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
        pickupMiddle.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),
        (
          scoreMiddleA.cmd().alongWith(
            WaitCommand(0.52).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )

          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )

    scoreMiddleA.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.RIGHT),
        robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
        pickupLeft.cmd().alongWith(GroundIntake(robot)),
        robot.drive.driveStop(),

        (
          scoreRightB.cmd().alongWith(
            WaitCommand(0.74).andThen(
              robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
            )
          )
          ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
      )
    )

    scoreRightB.done()
      .onTrue(
        Commands.sequence(
          ScoreL2PivotSide(robot, FieldConstants.ReefSide.RIGHT),
          robot.superstructureManager.requestGoal(SuperstructureGoal.PRE_GROUND),
          pickupRight.cmd().alongWith(GroundIntake(robot)),
          robot.drive.driveStop(),
          (
            scoreLeftA.cmd().alongWith(
              WaitCommand(0.68).andThen(
                robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE_PIVOTT)
              )
            )
            ) // .onlyIf { robot.intake.coralDetected() && RobotBase.isReal() }
        )
      )

    scoreLeftA.done()
      .onTrue(
        Commands.sequence(
          ScoreL2PivotSide(robot, FieldConstants.ReefSide.LEFT),
          robot.superstructureManager.requestGoal(SuperstructureGoal.STOW),
          robot.drive.driveStop(),

        )
      )

    return leftBack2l4l2
  }

  fun american_routine_optimal(): AutoRoutine {
    val opt_american = autoFactory.newRoutine("opt Ameriacn")
    val l4A_traj = opt_american.trajectory("GroundThreeHalf/L4A (I)")
    val l4B_traj = opt_american.trajectory("GroundThreeHalf/l4B")
    val loli1_traj = opt_american.trajectory("GroundThreeHalf/Loli 1")
    val loli2_traj = opt_american.trajectory("GroundThreeHalf/Loli 2")
    val l3_traj = opt_american.trajectory("GroundThreeHalf/l3B")
    val loli3_traj = opt_american.trajectory("GroundThreeHalf/Loli 3")

    opt_american.active().onTrue(
      l4A_traj.resetOdometry().andThen(l4A_traj.cmd())

    )

    l4A_traj.done().onTrue(
      Commands.sequence(
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.LEFT),
        loli1_traj.cmd().alongWith(GroundIntake(robot)),
        l4B_traj.cmd(),
        ScoreL4PivotSide(robot, FieldConstants.ReefSide.RIGHT),
        GroundIntake(robot).alongWith(loli2_traj.cmd()),
        l3_traj.cmd().andThen(ScoreL2PivotSide(robot, FieldConstants.ReefSide.RIGHT)),
        loli3_traj.cmd().alongWith(GroundIntake(robot))

      )
    )

    return opt_american
  }

  // Elevator is cooked!
  // autoChooser that will be displayed on dashboard
  fun addOptions(autoChooser: AutoChooser) {
    autoChooser.addRoutine("right 3.5 L4", this::rightGround3L4Half)
    autoChooser.addRoutine("left 3.5 fL4", this::leftGround3L4Half)

    autoChooser.addRoutine("RightBackL4+L2", this::rightGroundBack2L4l2)
    autoChooser.addRoutine("LeftBackL4+L2", this::leftGroundBack2L4l2)

    autoChooser.addRoutine("Left Middle&Sides", this::left3L4)
    autoChooser.addRoutine("Right Middle&Sides", this::right3L4)

    autoChooser.addRoutine("Taxi", this::taxi)
    autoChooser.addRoutine("The Goat", this::americanRoutine)
    // autoChooser.addRoutine("testing", this::middleRoutine)

    autoChooser.addRoutine("Left Goat", this::LeftamericanRoutine)
    autoChooser.addRoutine("optimal stuff", this::american_routine_optimal)
  }

  fun ScoreL4PivotSide(robot: Robot, reefSide: FieldConstants.ReefSide): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PIVOT)
      .alongWith(
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(reefSide), translationSpeedLim = 1.0, translationAccelLim = 1.95)
          .andThen(PrintCommand("Actually reached auto tolerance!"))
          .withTimeout(2.0)
      )
      .andThen(WaitCommand(0.10))
      .andThen(robot.intake.outtakeCoralPivot())
      .andThen(
        WaitUntilCommand { !robot.intake.coralDetected() }
          .onlyIf { RobotBase.isReal() }
      )
      .andThen(WaitCommand(0.050))
      .andThen(robot.intake.stop())
  }

  fun ScoreL3PivotSide(robot: Robot, reefSide: FieldConstants.ReefSide): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.L3)
      .alongWith(
        // robot.intake.outtakeAlgae(),
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(reefSide), translationSpeedLim = 1.0, translationAccelLim = 1.95)
          .andThen(PrintCommand("Actually reached auto tolerance!"))
          .withTimeout(2.0)
      )
      .andThen(WaitCommand(0.10))
      .andThen(robot.intake.outtakeCoralPivot())
      .andThen(
        WaitUntilCommand { !robot.intake.coralDetected() }
          .onlyIf { RobotBase.isReal() }
      )
      .andThen(WaitCommand(0.050))
      .andThen(robot.intake.stop())
  }

  fun ScoreL2PivotSide(robot: Robot, reefSide: FieldConstants.ReefSide): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.L2_PIVOT)
      .alongWith(
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(reefSide), translationSpeedLim = 1.0, translationAccelLim = 1.95)
          .andThen(PrintCommand("Actually reached auto tolerance!"))
          .withTimeout(2.0)
      )
      .andThen(WaitCommand(0.10))
      .andThen(robot.intake.outtakeCoralPivot())
      .andThen(
        WaitUntilCommand { !robot.intake.coralDetected() }
          .onlyIf { RobotBase.isReal() }
      )
      .andThen(WaitCommand(0.050))
      .andThen(robot.intake.stop())
  }

  fun Intake(robot: Robot): Command {
    return InstantCommand(robot.drive::stop)
      .andThen(robot.intake.intakeCoral())
      .andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE))
      .andThen(
        WaitUntilCommand { robot.intake.coralDetected() }
          .onlyIf { RobotBase.isReal() }
      )
      .andThen(robot.intake.stop())
  }

  fun PremoveIntake(robot: Robot): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.GROUND_INTAKE)
      .alongWith(robot.intake.intakeCoral())
  }

  fun GroundIntake(robot: Robot): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.GROUND_INTAKE)
      .alongWith(robot.intake.intakeCoral())
      .andThen(
        WaitUntilCommand { robot.intake.coralDetected() }
          .onlyIf { RobotBase.isReal() }
      )
      .andThen(
        robot.intake.stop()
      )
  }
}
