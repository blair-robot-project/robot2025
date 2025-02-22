package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import choreo.trajectory.SwerveSample
import dev.doglog.DogLog
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.Robot
import frc.team449.commands.driveAlign.SimpleReefAlign
import frc.team449.subsystems.FieldConstants
import frc.team449.subsystems.superstructure.SuperstructureGoal
import java.util.Optional

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
  var desiredAngle = 0.0
  var desiredOmega = 0.0

  private fun followTrajectory(sample: SwerveSample) {
    desiredAngle = MathUtil.angleModulus(sample.heading)
    desiredOmega = sample.omega
    val speeds = ChassisSpeeds(
      sample.vx + xController.calculate(robot.poseSubsystem.pose.x, sample.x),
      sample.vy + yController.calculate(robot.poseSubsystem.pose.y, sample.y),
      sample.omega + headingController.calculate(
        robot.poseSubsystem.pose.rotation.minus(Rotation2d.fromRadians(MathUtil.angleModulus(sample.heading))).radians
      ) // + if (DriverStation.getAlliance().getOrDefault(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) PI else 0.0, // Needs testing //0-2pi
// poseSubsystem  -pi- pi
    )
    val newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      speeds,
      robot.poseSubsystem.heading
    )

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



// L4 coral at reef G
  fun l4reefG(): AutoRoutine {
    val g: AutoRoutine = autoFactory.newRoutine("L4 reef G")
    val reefGTrajectory: AutoTrajectory = g.trajectory("middle_G")
    g.active().onTrue(
      Commands.sequence(
        reefGTrajectory.resetOdometry(),
        reefGTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.RIGHT)).alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
        ),
        robot.drive.driveStop().alongWith(robot.intake.outtakeCoral().until { (!robot.intake.coralDetected()) })
      )
    )
    return g
  }



  // L1 coral at reef G
  fun l1reefG(): AutoRoutine {
    val g: AutoRoutine = autoFactory.newRoutine("L1 reef G")
    val reefGTrajectory: AutoTrajectory = g.trajectory("middle_G")
    g.active().onTrue(
      Commands.sequence(
        reefGTrajectory.resetOdometry(),
        reefGTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE)),
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
        reefETrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.RIGHT)).alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
        ),
        robot.drive.driveStop().alongWith(robot.intake.outtakeCoral().until { (!robot.intake.coralDetected()) }),

        // to coral station
        robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
        reefEtoStationTrajectory.cmd().andThen(robot.drive.driveStop()),
        robot.intake.intakeCoral().until { robot.intake.coralDetected() },

        // to reef
        stationToDTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.RIGHT)).alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
        ),
        robot.drive.driveStop().alongWith(robot.intake.outtakeCoral().until { (!robot.intake.coralDetected()) }),
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

        // to reef
        reefJTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.RIGHT)).alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
        ),
        robot.drive.driveStop().alongWith(robot.intake.outtakeCoral().until { (!robot.intake.coralDetected()) }),

        // to coral station
        robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
        reefJtoStationTrajectory.cmd().andThen(robot.drive.driveStop()),
        robot.intake.intakeCoral().until { robot.intake.coralDetected() },

        // to reef
        stationToLTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.RIGHT)).alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
        ),
        robot.drive.driveStop().alongWith(robot.intake.outtakeCoral().until { (!robot.intake.coralDetected()) }),
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
        reefJTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.RIGHT)).alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
        ),
        robot.drive.driveStop().alongWith(robot.intake.outtakeCoral().until { (!robot.intake.coralDetected()) }),

        // to coral station
        robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
        reefJtoStationTrajectory.cmd().andThen(robot.drive.driveStop()),
        robot.intake.intakeCoral().until { robot.intake.coralDetected() },

        // to reef
        stationToKTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.RIGHT)).alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
        ),
        robot.drive.driveStop().alongWith(robot.intake.outtakeCoral().until { (!robot.intake.coralDetected()) }),

        // to coral station
        robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE),
        reefKtoStationTrajectory.cmd().andThen(robot.drive.driveStop()),
        robot.intake.intakeCoral().until { robot.intake.coralDetected() },

        // to reef
        stationToLTrajectory.cmd().alongWith(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)),
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.RIGHT)).alongWith(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
        ),
        robot.drive.driveStop().alongWith(robot.intake.outtakeCoral().until { (!robot.intake.coralDetected()) }),
      )
    )
    return J_K_L
  }


  fun logData() {
    DogLog.log("Autos/Current rotation:", robot.poseSubsystem.pose.rotation.radians)
    DogLog.log("Autos/Desired rotation:", desiredAngle)
    DogLog.log("Autos/Rotation Closed Loop Error:", robot.poseSubsystem.pose.rotation.minus(Rotation2d.fromRadians(MathUtil.angleModulus(desiredAngle))).radians)
    DogLog.log("Autos/Path Omega:", desiredOmega)
    DogLog.log(
      "Autos/Controller Effort:",
      headingController.calculate(
        robot.poseSubsystem.pose.rotation.minus(Rotation2d.fromRadians(MathUtil.angleModulus(desiredAngle))).radians
      )
    )
  }

  // autoChooser that will be displayed on dashboard
  fun addOptions(autoChooser: AutoChooser) {
    autoChooser.addRoutine("RightTaxi", this::rightTaxi)
    autoChooser.addRoutine("LeftTaxi", this::leftTaxi)
    autoChooser.addRoutine("l4 G", this::l4reefG)//middle l4
    autoChooser.addRoutine("l1 G", this::l1reefG)//middle l1
    autoChooser.addRoutine("l4 E & l4 D ", this::reefED) //2 piece l4
    autoChooser.addRoutine("l4 J & l4 L", this::reefJL) //2 piece l4
    autoChooser.addRoutine("l4 J,K,L", this::reefJKL)//3 piece l4
    autoChooser.addRoutine("The Goat", this::americanRoutine)//america
  }
}
