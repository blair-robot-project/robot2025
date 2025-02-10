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
import frc.team449.Robot/*
import frc.team449.subsystems.superstructure.SuperstructureGoal
//import frc.team449.subsystems.superstructure.intake.Intake*/
import kotlin.jvm.optionals.getOrDefault
import kotlin.math.PI

open class Routines(
  val robot: Robot) {
  private val xController: PIDController
    get() = PIDController(10.0, 0.0, 0.0)
  private val yController: PIDController
    get() = PIDController(10.0, 0.0, 0.0)
  private val headingController: PIDController
    get() = PIDController(6.7, 0.0, 0.0)

  init {
    headingController.enableContinuousInput(-Math.PI, Math.PI)
  }

  private fun followTrajectory(sample: SwerveSample) {
    val speeds = ChassisSpeeds(
      sample.vx + xController.calculate(robot.poseSubsystem.pose.x, sample.x),
      sample.vy + yController.calculate(robot.poseSubsystem.pose.y, sample.y),
      sample.omega + headingController.calculate(
        robot.poseSubsystem.pose.rotation.radians + if (DriverStation.getAlliance().getOrDefault(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) PI else 0.0,
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
    lTaxi.active().onTrue(Commands.sequence(lTaxiTrajectory.resetOdometry(), lTaxiTrajectory.cmd(),robot.drive.driveStop()))
    return lTaxi
  }


// l1 coral at reef E
  fun l1reefE(): AutoRoutine {
    val l1E: AutoRoutine = autoFactory.newRoutine("L1 reef E")
    val reefETrajectory: AutoTrajectory = l1E.trajectory("right_E")
    l1E.active().onTrue(
      Commands.sequence(
        reefETrajectory.resetOdometry(),
        reefETrajectory.cmd(),
        robot.drive.driveStop(),
      )
    )
    return l1E
  }


  //l4 coral at reef E
  fun l4reefE(): AutoRoutine {
    val l1E: AutoRoutine = autoFactory.newRoutine("L4 reef E")
    val reefETrajectory: AutoTrajectory = l1E.trajectory("right_E")
    l1E.active().onTrue(
      Commands.sequence(
        reefETrajectory.resetOdometry(),
        reefETrajectory.cmd(),
            robot.drive.driveStop()
      )
    )
    return l1E
  }


  //L1 coral at reef F
  fun l1reefF(): AutoRoutine {
    val F: AutoRoutine = autoFactory.newRoutine("L4 reef F")
    val reefFTrajectory: AutoTrajectory = F.trajectory("left_F")
    F.active().onTrue(
      Commands.sequence(
        reefFTrajectory.resetOdometry(),
         reefFTrajectory.cmd(),
        robot.drive.driveStop()
      )
    )
    return F
  }


//L4 coral at reef G
  fun l4reefG(): AutoRoutine {
    val g: AutoRoutine = autoFactory.newRoutine("L4 reef G")
    val reefGTrajectory: AutoTrajectory = g.trajectory("middle_G")
    g.active().onTrue(
      Commands.sequence(
        reefGTrajectory.resetOdometry(),
         reefGTrajectory.cmd(),
        robot.drive.driveStop()
      )
    )
    return g
  }

  //l1 coral at reef J
  fun l1reefJ(): AutoRoutine {
    val J: AutoRoutine = autoFactory.newRoutine("L1 reef J")
    val reefJTrajectory: AutoTrajectory = J.trajectory("left_J")
    J.active().onTrue(
      Commands.sequence(
        reefJTrajectory.resetOdometry(),
      reefJTrajectory.cmd(),
        robot.drive.driveStop()
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
        reefETrajectory.cmd(),
        robot.drive.driveStop(),
        reefEtoStationTrajectory.cmd(),
        robot.drive.driveStop(),
        stationToDTrajectory.cmd(),
        robot.drive.driveStop()),
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
        reefJTrajectory.cmd(),
        robot.drive.driveStop(),
        reefJtoStationTrajectory.cmd(),
        robot.drive.driveStop(),
        stationToLTrajectory.cmd(),
      robot.drive.driveStop()
      )
    )
    return J_L
  }

  // l1 at reef J and then l4 at reef K
  fun reefJK(): AutoRoutine {
    val J_K: AutoRoutine = autoFactory.newRoutine("L1 reef J and L4 reef K")
    val reefJTrajectory: AutoTrajectory = J_K.trajectory("left_J")
    val reefJtoStationTrajectory: AutoTrajectory = J_K.trajectory("J_leftStation")
    val stationToKTrajectory: AutoTrajectory = J_K.trajectory("leftStation_K")
    J_K.active().onTrue(

      Commands.sequence(
        reefJTrajectory.resetOdometry(),
        reefJTrajectory.cmd(),
        robot.drive.driveStop(),
        reefJtoStationTrajectory.cmd(),
        robot.drive.driveStop(),
        stationToKTrajectory.cmd(),
        robot.drive.driveStop()
      )
    )
    return J_K
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
        reefJTrajectory.cmd(),
        robot.drive.driveStop(),
        reefJtoStationTrajectory.cmd(),
        robot.drive.driveStop(),
        stationToKTrajectory.cmd(),
        robot.drive.driveStop(),
        reefKtoStationTrajectory.cmd(),
        robot.drive.driveStop(),
        stationToLTrajectory.cmd(),
        robot.drive.driveStop()
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
        reefJTrajectory.cmd(),
        robot.drive.driveStop(),
        reefJtoStationTrajectory.cmd(),
        robot.drive.driveStop(),
        stationToKTrajectory.cmd(),
        robot.drive.driveStop(),
        reefKtoStationTrajectory.cmd(),
        robot.drive.driveStop(),
        stationToKTrajectory.cmd(),
        robot.drive.driveStop(),
        reefKtoStationTrajectory.cmd(),
        robot.drive.driveStop(),
        stationToKTrajectory.cmd(),
        robot.drive.driveStop()
      )
    )
    return J_fullK
  }





  //left side of the field spam l1
  fun leftSpamL1():AutoRoutine{
    val spam : AutoRoutine = autoFactory.newRoutine(" spamming")
    val reefJTrajectory: AutoTrajectory = spam.trajectory("left_J")
    val reefJToLeftStation: AutoTrajectory = spam.trajectory("J_leftStation")
    val leftStationToReefK: AutoTrajectory = spam.trajectory("leftStation_K")
    val reefKToLeftStation: AutoTrajectory = spam.trajectory("K_leftStation")
    val leftStationReefL : AutoTrajectory = spam.trajectory("leftStation_L")
    val reefLtoLeftStation : AutoTrajectory = spam.trajectory("L_leftStation")
    spam.active().onTrue(
      Commands.sequence(

        reefJTrajectory.resetOdometry(),
        reefJTrajectory.cmd(),
        robot.drive.driveStop(),
        reefJToLeftStation.cmd(),
        robot.drive.driveStop(),
        leftStationToReefK.cmd(),
        robot.drive.driveStop(),
        reefKToLeftStation.cmd(),
        robot.drive.driveStop(),
        leftStationReefL.cmd(),
        robot.drive.driveStop(),
        reefLtoLeftStation.cmd(),
        robot.drive.driveStop(),
        leftStationToReefK.cmd(),
        robot.drive.driveStop(),
        reefKToLeftStation.cmd(),
        robot.drive.driveStop(),
        leftStationReefL.cmd(), robot.drive.driveStop()
        )
    )
    return spam
  }




  //left side of the field spam l1
  fun rightSpamL1():AutoRoutine{
    val spam : AutoRoutine = autoFactory.newRoutine(" spamming")
    val reefETrajectory: AutoTrajectory = spam.trajectory("right_E")
    val reefEToRightStation: AutoTrajectory = spam.trajectory("E_rightStation")
    val rightStationToReefD: AutoTrajectory = spam.trajectory("rightStation_D")
    val reefDToRightStation: AutoTrajectory = spam.trajectory("D_rightStation")
    val rightStationReefC : AutoTrajectory = spam.trajectory("rightStation_C")
    val reefCtoRightStation : AutoTrajectory = spam.trajectory("C_rightStation")
    spam.active().onTrue(
      Commands.sequence(

        reefETrajectory.resetOdometry(),
       reefETrajectory.cmd(), robot.drive.driveStop(), Commands.waitSeconds(0.5),
        reefEToRightStation.cmd(), robot.drive.driveStop(),Commands.waitSeconds(0.5),
        rightStationToReefD.cmd(), robot.drive.driveStop(),Commands.waitSeconds(0.5),
        reefDToRightStation.cmd(), robot.drive.driveStop(),Commands.waitSeconds(0.5),
        rightStationReefC.cmd(), robot.drive.driveStop(),Commands.waitSeconds(0.5),
        reefCtoRightStation.cmd(), robot.drive.driveStop(),Commands.waitSeconds(0.5),
        rightStationToReefD.cmd(), robot.drive.driveStop(),Commands.waitSeconds(0.5),
        reefDToRightStation.cmd(), robot.drive.driveStop(),Commands.waitSeconds(0.5),
        rightStationReefC.cmd(), robot.drive.driveStop()

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
    autoChooser.addRoutine("l1 J & l4 K", this::reefJK)
    autoChooser.addRoutine("l4 J,K,L", this::reefJKL)
    autoChooser.addRoutine("l1_J + full K", this::reefJ_fullReefK)
    autoChooser.addRoutine("l1 rightSpam", this::rightSpamL1)
    autoChooser.addRoutine("l1 leftSpam", this::leftSpamL1)
  }
}