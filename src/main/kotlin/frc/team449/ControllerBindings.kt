package frc.team449

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.team449.commands.driveAlign.SimpleReefAlign
import frc.team449.subsystems.FieldConstants
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveSim
import frc.team449.subsystems.drive.swerve.WheelRadiusCharacterization
import frc.team449.subsystems.superstructure.SuperstructureGoal
import frc.team449.subsystems.superstructure.intake.IntakeConstants
import frc.team449.subsystems.superstructure.wrist.WristConstants
import java.util.Optional
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI
import kotlin.random.Random

class ControllerBindings(
  private val driveController: CommandXboxController,
  private val mechanismController: CommandXboxController,
  private val characterizationController: CommandXboxController,
  private val testController: CommandXboxController,
  private val robot: Robot
) {

  val percentageElevatorPosition = { robot.elevator.positionSupplier.get() / SuperstructureGoal.L4.elevator.`in`(Meters) }
  private fun robotBindings() {
    /** Call robot functions you create below */
    /** Driver: https://docs.google.com/drawings/d/13W3qlIxzIh5MTraZGWON7IqwJvovVr8eNBvjq8_vYZI/edit
     * Operator: https://docs.google.com/drawings/d/1lF4Roftk6932jMCQthgKfoJVPuTVSgnGZSHs5j68uo4/edit
     */
    processor()
    println("processor")
    scoreIntakeL2()
    println("l2")
    scoreIntakeL3()
    println("l3")
    scoreL4Net()
    println("net")

    autoScoreLeft()
    autoScoreRight()

    groundIntakeVertical()
    groundIntakeL1andOuttake()
    algaeGroundIntake()

    stow()
    climbTriggers()
    climbwheels()

    stopReefAlign()

    manualElevator()
    manualPivot()
    manualWrist()

    manualCoral()
    manualAlgae()
  }
  private fun characterizationBindings() {
//    testVoltagePivot()
//    runClimbWheels()
//    pivotCharacterizaton()
  }
  private fun nonRobotBindings() {
     slowDrive()
    /** NOTE: If you want to see simulated vision convergence times with this function, go to simulationPeriodic in
     * RobotBase and change the passed in pose to it.simulationPeriodic to robot.drive.odometryPose
     */
//    if (RobotBase.isSimulation()) resetOdometrySim()

    resetGyro()
  }

/** driver controller TRIGGERS **/
  private fun autoScoreLeft() {
    driveController.leftTrigger().onTrue(
      ConditionalCommand(
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.RIGHT)),
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.LEFT))
      ) { robot.poseSubsystem.isBackReefSide() && robot.poseSubsystem.isPivotSide() }
    ).onFalse(
      robot.driveCommand
    )
  }
  private fun autoScoreRight() {
    driveController.rightTrigger().onTrue(
      ConditionalCommand(
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.LEFT)),
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.RIGHT))
      ) { robot.poseSubsystem.isBackReefSide() && robot.poseSubsystem.isPivotSide() }
    ).onFalse(
      robot.driveCommand
    )
  }

/** driver controller BUMPER **/
  private fun groundIntakeVertical() {
    driveController.leftBumper().onTrue(
      Commands.sequence(
        runOnce ({ robot.intake.resetPos() }),
        Commands.parallel(
          robot.superstructureManager.requestGoal(SuperstructureGoal.GROUND_INTAKE_CORAL),
          robot.intake.intakeToVertical()
        ),
        robot.superstructureManager.requestGoal(SuperstructureGoal.STOW),
        robot.intake.recenterCoral()
      )
    )
  }

/** driver controller START and BACK **/
  private fun algaeGroundIntake() {
    driveController.start().onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.ALGAE_GROUND)
        .alongWith(robot.intake.intakeAlgae())
        .andThen(WaitCommand(0.15))
        .andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW))
    )
  }

  private fun groundIntakeL1andOuttake() {
    driveController.rightBumper().onTrue(
      ConditionalCommand(

        // outtake
        Commands.sequence(

          ConditionalCommand( // currently have coral

            ConditionalCommand( // outtaking to l1
              robot.intake.outtakeL1(),

              ConditionalCommand( // scoring on pivot side
                robot.intake.outtakeCoralPivot(),
                robot.intake.outtakeCoral()
              ) { robot.superstructureManager.requestedPivotSide() }

            ) { robot.superstructureManager.lastRequestedGoal() == SuperstructureGoal.L1 },

            // dont have coral
            robot.intake.outtakeAlgae()

          ) { robot.intake.coralDetected() }.onlyIf { RobotBase.isReal() },
          // intake subsystem will stop motors automatically when finished outtaking so we don't need to do that here
          WaitCommand(0.15),
          robot.superstructureManager.requestRetraction(SuperstructureGoal.STOW)
            .onlyIf {
              robot.superstructureManager.lastRequestedGoal() != SuperstructureGoal.L1 &&
                robot.superstructureManager.lastRequestedGoal() != SuperstructureGoal.PROC
            }

        ),

        // intake l1
        Commands.sequence(
          runOnce ({ robot.intake.resetPos() }),
          Commands.parallel(
            robot.superstructureManager.requestGoal(SuperstructureGoal.GROUND_INTAKE_CORAL),
            robot.intake.intakeToHorizontal()
          ),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L1),
        )

      ) { robot.intake.hasPiece() }
    )
  }

/** driver controller A,B,X,Y **/
  private fun processor() {
    Trigger {
      driveController.hid.aButton &&
        robot.intake.algaeDetected()
    }.onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.PROC).alongWith(
        robot.intake.holdAlgaeProc()

      )
    )
  }
  private fun climbTriggers() {
    Trigger {
      driveController.hid.aButton &&
        !robot.intake.hasPiece()
    }.onTrue(
      Commands.sequence(
        robot.elevator.manualDown()
          .withDeadline(WaitCommand(0.350)),
        robot.superstructureManager.requestGoal(SuperstructureGoal.CLIMB_BEFORE)
          .alongWith(robot.climb.runClimbWheels()),
        robot.climb.waitUntilCurrentSpike(),
        WaitCommand(0.3276),
        Commands.parallel(
          robot.wrist.setPosition(WristConstants.STARTUP_ANGLE.`in`(Radians)),
          robot.climb.stop(),
          robot.pivot.climbDown(),
          WaitUntilCommand { robot.pivot.climbReady() }
            .andThen(robot.elevator.climbDown())
        ),
        WaitCommand(0.5).andThen(robot.climb.stop())
      )
    )
  }

  private fun scoreIntakeL2() {
    driveController.x().onTrue(
      ConditionalCommand(

        ConditionalCommand(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L2_PIVOT)
            .alongWith(robot.intake.moveCoralPivotSide()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L2)
            .alongWith(robot.intake.moveCoralOppSide())
        ) { robot.poseSubsystem.isPivotSide() },

        robot.superstructureManager.requestGoal(SuperstructureGoal.L2_ALGAE_INTAKE)
          .alongWith(robot.intake.intakeAlgae())
          .andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW))
      ) { robot.intake.coralDetected() }
    )
  }

  private fun scoreIntakeL3() {
    driveController.b().onTrue(
      ConditionalCommand(

        ConditionalCommand(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L3_PIVOT)
            .alongWith(robot.intake.moveCoralPivotSide()),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L3)
            .alongWith(robot.intake.moveCoralOppSide())
        ) { robot.poseSubsystem.isPivotSide() },

        robot.superstructureManager.requestGoal(SuperstructureGoal.L3_ALGAE_INTAKE)
          .alongWith(robot.intake.intakeAlgae())
          .andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW))
      ) { robot.intake.coralDetected() }
    )
  }

  private fun scoreL4Net() {
    driveController.y().onTrue(
      ConditionalCommand(

        ConditionalCommand(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PIVOT)
            .alongWith(robot.intake.moveCoralPivotSide()),

          robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
            .alongWith(robot.intake.moveCoralOppSide())

        ) { robot.poseSubsystem.isPivotSide() },

        ConditionalCommand(
          robot.superstructureManager.requestGoal(SuperstructureGoal.NET),
          robot.superstructureManager.requestGoal(SuperstructureGoal.NET_PIVOT)
        ) { robot.poseSubsystem.isFacingNet() }

      ) { robot.intake.coralDetected() }
    )
  }

/** driver controller dpad **/
  // povUp
  private fun resetGyro() {
    driveController.povUp().onTrue(
      ConditionalCommand(
        InstantCommand({ robot.poseSubsystem.heading = Rotation2d(PI) }),
        InstantCommand({ robot.poseSubsystem.heading = Rotation2d() })
      ) { DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red }
    )
  }

  // povDown
  private fun stow() {
    driveController.povDown().onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
        .deadlineFor(robot.light.progressMaskGradient(percentageElevatorPosition))
        .alongWith(robot.climb.stop())
        .alongWith(
          ConditionalCommand(
            robot.intake.holdAlgae() ,
            robot.intake.stopMotors()
          ){ robot.intake.algaeDetected() })

        .andThen(ConditionalCommand(
          robot.intake.stopMotors(),
            robot.intake.holdAlgae()
        ){ robot.intake.topMotor.statorCurrent.valueAsDouble <
          10.0}))


  }

  /** mech controller TRIGGERS **/
  private fun climbwheels() {
    mechanismController.leftTrigger().onTrue(
      robot.climb.runClimbWheels()
    ).onFalse(
      robot.climb.stop()
    )
  }

/** mech controller BUMPERS **/
  private fun manualWrist() {
    // up
    mechanismController.rightBumper().onTrue(
      robot.wrist.manualUp()
    ).onFalse(robot.wrist.hold())

    // down
    mechanismController.leftBumper().onTrue(
      robot.wrist.manualDown()
    ).onFalse(robot.wrist.hold())
  }

/** mech controller START and BACK **/
  private fun stopReefAlign() {
    mechanismController.start().onTrue(
      robot.driveCommand
    )

    mechanismController.back().onTrue(
      robot.intake.intakeToHorizontal()
    ).onFalse(robot.intake.stopMotors())
  }

/** mech controller A,B,X,Y **/
  private fun manualAlgae() {
    // intake algae
    mechanismController.a().onTrue(
      robot.intake.intakeAlgae()
    )

    // outtake algae
    mechanismController.b().onTrue(
      robot.intake.outtakeAlgae()
    )
  }
  private fun manualCoral() {
    // intake coral
    mechanismController.x().onTrue(
      robot.intake.manualIn()
    ).onFalse(robot.intake.stopMotors())

    // outtake coral
    mechanismController.y().onTrue(
      robot.intake.manualOut()
    ).onFalse(robot.intake.stopMotors())
  }

/** mech controller dpad **/
  private fun manualPivot() {
    // up
    mechanismController.povLeft().onTrue(
      robot.pivot.manualUp()
    ).onFalse(robot.pivot.hold())
    // down
    mechanismController.povRight().onTrue(
      robot.pivot.manualDown()
    ).onFalse(robot.pivot.hold())
  }
  private fun manualElevator() {
    // up
    mechanismController.povUp().onTrue(
      robot.elevator.manualUp()
    ).onFalse(robot.elevator.hold())

    // down
    mechanismController.povDown().onTrue(
      robot.elevator.manualDown()
    ).onFalse(robot.elevator.hold())
  }

  private fun autoScoreLeftOuttake() {
    driveController.leftTrigger().onTrue(
      Commands.sequence(
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.LEFT))
          .deadlineFor(robot.light.gradient(MetersPerSecond.of(0.35), Color.kPurple, Color.kWhite)),
        Commands.parallel(
          robot.intake.outtakeCoral()
            .andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW))
            .onlyIf {
              robot.superstructureManager.lastRequestedGoal() == SuperstructureGoal.L3 ||
                robot.superstructureManager.lastRequestedGoal() == SuperstructureGoal.L2
            },
          robot.light.blink(Seconds.of(0.20), Color.kWhite)
            .withTimeout(1.5)
        )
      )
    ).onFalse(
      robot.driveCommand
    )
  }
  private fun autoScoreRightOuttake() {
    driveController.rightTrigger().onTrue(
      Commands.sequence(
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.LEFT))
          .deadlineFor(robot.light.gradient(MetersPerSecond.of(0.35), Color.kPurple, Color.kWhite)),
        Commands.parallel(
          robot.intake.outtakeCoral()
            .andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW))
            .onlyIf {
              robot.superstructureManager.lastRequestedGoal() == SuperstructureGoal.L3 ||
                robot.superstructureManager.lastRequestedGoal() == SuperstructureGoal.L2
            },
          robot.light.blink(Seconds.of(0.20), Color.kWhite)
            .withTimeout(1.5)
        )
      )
    ).onFalse(
      robot.driveCommand
    )
  }
  private fun slowDrive() {
    driveController.rightBumper().onTrue(
      InstantCommand({ robot.drive.maxLinearSpeed = 1.0 })
        .andThen(InstantCommand({ robot.drive.maxRotSpeed = PI / 2 }))
    ).onFalse(
      InstantCommand({ robot.drive.maxLinearSpeed = RobotConstants.MAX_LINEAR_SPEED })
        .andThen(
          InstantCommand({ robot.drive.maxRotSpeed = RobotConstants.MAX_ROT_SPEED })
        )
    )
  }
  private fun resetOdometrySim() {
    driveController.a().onTrue(
      InstantCommand({
        robot.drive as SwerveSim
        robot.drive.resetOdometryOnly(
          Pose2d(
            robot.drive.odometryPose.x + Random.nextDouble(-1.0, 1.0),
            robot.drive.odometryPose.y + Random.nextDouble(-1.0, 1.0),
            robot.drive.odometryPose.rotation
          )
        )
      })
    )
  }
  private fun pointToRight() {
    driveController.a().onTrue(
      robot.driveCommand.pointAtAngleCommand(Rotation2d.fromDegrees(90.0))
    )
  }

  // pivot test
  private fun testVoltagePivot() {
    characterizationController.rightTrigger().onTrue(
      robot.pivot.testVoltage()
    ).onFalse(
      robot.pivot.hold()
    )
  }

  /** Characterization functions */
  private fun wheelRadiusCharacterization() {
    characterizationController.leftTrigger().onTrue(
      WheelRadiusCharacterization(robot.drive, robot.poseSubsystem)
    )
  }
  private fun driveCharacterization() {
    val driveRoutine = SysIdRoutine(
      SysIdRoutine.Config(
        Volts.of(1.0).per(Second),
        Volts.of(2.0),
        Seconds.of(4.0)
      ) { state -> SignalLogger.writeString("state", state.toString()) },
      Mechanism(
        { voltage: Voltage -> robot.drive.setVoltage(-voltage.`in`(Volts)) },
        null,
        robot.drive
      )
    )

    // Quasistatic Forwards
    characterizationController.povUp().onTrue(
      driveRoutine.quasistatic(SysIdRoutine.Direction.kForward)
    )

    // Quasistatic Reverse
    characterizationController.povDown().onTrue(
      driveRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
    )

    // Dynamic Forwards
    characterizationController.povRight().onTrue(
      driveRoutine.dynamic(SysIdRoutine.Direction.kForward)
    )

    // Dynamic Reverse
    characterizationController.povLeft().onTrue(
      driveRoutine.dynamic(SysIdRoutine.Direction.kReverse)
    )
  }
  private fun elevatorCharacterizaton() {
    val elevatorRoutine = SysIdRoutine(
      SysIdRoutine.Config(
        Volts.of(0.35).per(Second),
        Volts.of(1.5),
        Seconds.of(10.0)
      ) { state -> SignalLogger.writeString("state", state.toString()) },
      Mechanism(
        { voltage: Voltage -> robot.elevator.setVoltage(voltage.`in`(Volts)) },
        null,
        robot.elevator,
        "elevator"
      )
    )

    characterizationController.povUp().onTrue(
      elevatorRoutine.quasistatic(SysIdRoutine.Direction.kForward)
    )

    characterizationController.povDown().onTrue(
      elevatorRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
    )

    characterizationController.povRight().onTrue(
      elevatorRoutine.dynamic(SysIdRoutine.Direction.kForward)
    )

    characterizationController.povLeft().onTrue(
      elevatorRoutine.dynamic(SysIdRoutine.Direction.kReverse)
    )
  }
  private fun pivotCharacterizaton() {
    val pivotRoutine = SysIdRoutine(
      SysIdRoutine.Config(
        Volts.of(0.5).per(Second),
        Volts.of(2.0),
        Seconds.of(10.0)
      ) { state -> SignalLogger.writeString("state", state.toString()) },
      Mechanism(
        { voltage: Voltage -> robot.pivot.setVoltageChar(-voltage.`in`(Volts)) },
        null,
        robot.pivot,
        "elevator"
      )
    )

    characterizationController.povUp().onTrue(
      pivotRoutine.quasistatic(SysIdRoutine.Direction.kForward)
    )

    characterizationController.povDown().onTrue(
      pivotRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
    )

    characterizationController.povRight().onTrue(
      pivotRoutine.dynamic(SysIdRoutine.Direction.kForward)
    )

    characterizationController.povLeft().onTrue(
      pivotRoutine.dynamic(SysIdRoutine.Direction.kReverse)
    )
  }
  private fun wristCharacterizaton() {
    val wristRoutine = SysIdRoutine(
      SysIdRoutine.Config(
        Volts.of(0.35).per(Second),
        Volts.of(1.25),
        Seconds.of(10.0)
      ) { state -> SignalLogger.writeString("state", state.toString()) },
      Mechanism(
        { voltage: Voltage -> robot.wrist.setVoltageChar(voltage.`in`(Volts)) },
        null,
        robot.wrist,
        "wrist"
      )
    )

    characterizationController.povUp().onTrue(
      wristRoutine.quasistatic(SysIdRoutine.Direction.kForward).alongWith(
        PrintCommand("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
      )
    )

    characterizationController.povDown().onTrue(
      wristRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
    )

    characterizationController.povRight().onTrue(
      wristRoutine.dynamic(SysIdRoutine.Direction.kForward)
    )

    characterizationController.povLeft().onTrue(
      wristRoutine.dynamic(SysIdRoutine.Direction.kReverse)
    )
  }

  /** Try not to touch, just add things to the robot or nonrobot bindings */
  fun bindButtons() {
    println("Binding Buttons")
    nonRobotBindings()
    println("\tBound non robot bindings")
    robotBindings()
    println("\tBound robot bindings")
    characterizationBindings()
    println("\tBound characterization bindings")
  }
}
