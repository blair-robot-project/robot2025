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
import frc.team449.Robot
import frc.team449.subsystems.superstructure.SuperstructureGoal
//import frc.team449.subsystems.superstructure.intake.Intake
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
    robot.drive.driveRobotRelative(newSpeeds)
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

  // right taxi
  fun rightTaxi(): AutoRoutine {
    val rTaxi: AutoRoutine = autoFactory.newRoutine("Right Taxi")
    val rTaxiTrajectory: AutoTrajectory = rTaxi.trajectory("taxiRight")
    rTaxi.active().onTrue(
      Commands.sequence(
        rTaxiTrajectory.resetOdometry(),
        rTaxiTrajectory.cmd(),
        robot.drive.driveStop())
    )
    return rTaxi
  }

  // left taxi
  fun leftTaxi(): AutoRoutine {
    val lTaxi: AutoRoutine = autoFactory.newRoutine("Left Taxi")
    val lTaxiTrajectory: AutoTrajectory = lTaxi.trajectory("taxiLeft")
    lTaxi.active().onTrue(
      Commands.sequence(
        lTaxiTrajectory.resetOdometry(),
        lTaxiTrajectory.cmd(),
        robot.drive.driveStop()
      )
    )
    return lTaxi
  }

  // coral at l1 one on reef E
  fun l1reefE(): AutoRoutine {
    val l1E: AutoRoutine = autoFactory.newRoutine("L1 reef E")
    val reefETrajectory: AutoTrajectory = l1E.trajectory("right_E")
    l1E.active().onTrue(
      Commands.sequence(
        reefETrajectory.resetOdometry(),
        reefETrajectory.cmd(),
        robot.drive.driveStop(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE),
        robot.intake.outtakeCoral()

        )
    )
    return l1E
  }

  // coral at l4 one on reef F
  fun l1reefF(): AutoRoutine {
    val F: AutoRoutine = autoFactory.newRoutine("L4 reef F")
    val reefFTrajectory: AutoTrajectory = F.trajectory("left_F")
    F.active().onTrue(
      Commands.sequence(
        reefFTrajectory.resetOdometry(),
          reefFTrajectory.cmd(),
          robot.drive.driveStop(),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE),
          robot.intake.outtakeCoral()

      )
    )
    return F
  }



  // coral at l4 one on reef G
  fun reefG(): AutoRoutine {
    val g: AutoRoutine = autoFactory.newRoutine("L4 reef G")
    val reefGTrajectory: AutoTrajectory = g.trajectory("middle_G")
    g.active().onTrue(
      Commands.sequence(
        reefGTrajectory.resetOdometry(),
        reefGTrajectory.cmd(),
          robot.drive.driveStop(),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE),
          robot.intake.outtakeCoral()
      )
    )
    return g
  }

  // coral at l1 one on reef J
  fun reefJ(): AutoRoutine {
    val J: AutoRoutine = autoFactory.newRoutine("L1 reef J")
    val reefJTrajectory: AutoTrajectory = J.trajectory("left_J")
    J.active().onTrue(
      Commands.sequence(
        reefJTrajectory.resetOdometry(),
          reefJTrajectory.cmd(),
        robot.drive.driveStop(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE),
        robot.intake.intakeCoral()

        )
    )
    return J
  }

  // placing L4 at reef E then l1 at reef D
  fun reefED(): AutoRoutine {
    val E_D: AutoRoutine = autoFactory.newRoutine("L1 reef E and L4 reef D")
    val reefETrajectory: AutoTrajectory = E_D.trajectory("right_E")
    val reefEtoStationTrajectory: AutoTrajectory = E_D.trajectory("E_rightStation")
    val stationToDTrajectory: AutoTrajectory = E_D.trajectory("rightStation_D")
    E_D.active().onTrue(

      // go to reef e and score l1
      Commands.sequence(
        reefETrajectory.resetOdometry(),
          reefETrajectory.cmd(),
          robot.drive.driveStop(),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE),
          robot.intake.outtakeCoral(),

        //go to the coral station and intake coral

            reefEtoStationTrajectory.cmd(),
          robot.drive.driveStop(),
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
          robot.intake.intakeCoral(),

        //go to reef d from station and score l4

            stationToDTrajectory.cmd(),
            robot.drive.driveStop(),
            robot.superstructureManager.requestGoal(SuperstructureGoal.L4_fromStation),
            robot.intake.outtakeCoral(),

        )
      )

    return E_D
  }

  // placing L4 at reef E then l1 at reef L
  fun reefJL(): AutoRoutine {
    val J_L: AutoRoutine = autoFactory.newRoutine("L4 reef J and L1 reef L")
    val reefJTrajectory: AutoTrajectory = J_L.trajectory("left_J")
    val reefJtoStationTrajectory: AutoTrajectory = J_L.trajectory("J_leftStation")
    val stationToLTrajectory: AutoTrajectory = J_L.trajectory("leftStation_L")
    J_L.active().onTrue(
      Commands.sequence(
        reefJTrajectory.resetOdometry(),

          reefJTrajectory.cmd(),
          robot.drive.driveStop(),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE),
          robot.intake.outtakeCoral(),

          reefJtoStationTrajectory.cmd(),
          robot.drive.driveStop(),
          robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
          robot.intake.intakeAlgae(),

          stationToLTrajectory.cmd(),
          robot.drive.driveStop(),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_fromStation),
          robot.intake.outtakeCoral()


        )
      )

    return J_L
  }

  fun reefJK(): AutoRoutine {
    val J_K: AutoRoutine = autoFactory.newRoutine("L4 reef J and L1 reef K")
    val reefJTrajectory: AutoTrajectory = J_K.trajectory("left_J")
    val reefJtoStationTrajectory: AutoTrajectory = J_K.trajectory("J_leftStation")
    val stationToKTrajectory: AutoTrajectory = J_K.trajectory("leftStation_K")
    J_K.active().onTrue(
      Commands.sequence(
        reefJTrajectory.resetOdometry(),
        reefJTrajectory.cmd(),
        robot.drive.driveStop(),
        robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE),
        robot.intake.outtakeCoral(),

          reefJtoStationTrajectory.cmd(),
          robot.drive.driveStop(),
         robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
         robot.intake.intakeCoral(),


          stationToKTrajectory.cmd(),
          robot.drive.driveStop(),
         robot.superstructureManager.requestGoal(SuperstructureGoal.L4_fromStation),
        robot.intake.outtakeCoral()

        )
      )

    return J_K
  }

  // autoChooser that will be displayed on dashboard
  fun addOptions(autoChooser: AutoChooser) {
    autoChooser.addRoutine("Nothing", this::doNothing)
    autoChooser.addRoutine("RightTaxi", this::rightTaxi)
    autoChooser.addRoutine("LeftTaxi", this::leftTaxi)
    autoChooser.addRoutine("l1 E", this::l1reefE)
    autoChooser.addRoutine("l4 F", this::l1reefF)
    autoChooser.addRoutine("l4 G", this::reefG)
    autoChooser.addRoutine("l1 J", this::reefJ)
    autoChooser.addRoutine("l1 E & l4 D ", this::reefED)
    autoChooser.addRoutine("l1 J & l4 L", this::reefJL)
    autoChooser.addRoutine("l1 J & l4 K", this::reefJK)
  }
}

