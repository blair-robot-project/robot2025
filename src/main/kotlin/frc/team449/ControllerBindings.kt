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
import frc.team449.subsystems.superstructure.wrist.WristConstants
import java.util.Optional
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI
import kotlin.random.Random

class ControllerBindings(
  private val driveController: CommandXboxController,
  private val mechanismController: CommandXboxController,
  private val characterizationController: CommandXboxController,
  private val robot: Robot
) {

  val percentageElevatorPosition = { robot.elevator.positionSupplier.get() / SuperstructureGoal.L4.elevator.`in`(Meters) }

  private fun robotBindings() {
    /** Call robot functions you create below */
    /** Driver: https://docs.google.com/drawings/d/13W3qlIxzIh5MTraZGWON7IqwJvovVr8eNBvjq8_vYZI/edit
     * Operator: https://docs.google.com/drawings/d/1lF4Roftk6932jMCQthgKfoJVPuTVSgnGZSHs5j68uo4/edit
     */
    score_l1()
    scoreDescore_l2()
    scoreDescore_l3()
    score_l4()

    autoScoreLeft()
    autoScoreRight()
//    autoScoreStowTrigger()

//    substationIntake() // replace with groundIntake() soon
    groundIntake()
//    coralBlockSubstationIntake()
    outtake()

    stow()
    climbTriggers()
    climbwheels()
    scoreL2()
    scoreL3()
    stopReefAlign()

    manualElevator()
    manualPivot()
    manualWrist()
//    intakeAlgae()
    intakeCoral()
    outtakeCoral()
  }

  private fun characterizationBindings() {
//    testVoltagePivot()
//    runClimbWheels()

//    pivotCharacterizaton()
  }

  private fun nonRobotBindings() {
    // slowDrive()

    /** NOTE: If you want to see simulated vision convergence times with this function, go to simulationPeriodic in
     * RobotBase and change the passed in pose to it.simulationPeriodic to robot.drive.odometryPose
     */
//    if (RobotBase.isSimulation()) resetOdometrySim()

    resetGyro()
  }

  private fun climbwheels() {
    mechanismController.leftTrigger().onTrue(
      robot.climb.runClimbWheels()
    ).onFalse(
      robot.climb.stop()
    )
  }

  private fun climbTriggers() {
//    Trigger {
//      driveController.hid.aButton &&
//        !robot.intake.coralDetected() &&
//        robot.superstructureManager.lastRequestedGoal() == SuperstructureGoal.CLIMB_BEFORE &&
//        robot.superstructureManager.isAtPos()
//    }.onTrue(
//      robot.wrist.setPosition(WristConstants.CLIMB_DOWN.`in`(Radians))
//        .alongWith(robot.climb.stop())
//        .alongWith(robot.pivot.climbDown())
//        .andThen(robot.elevator.climbDown())
//    )
//
//    Trigger {
//      driveController.hid.aButton &&
//        !robot.intake.coralDetected() &&
//        robot.superstructureManager.lastRequestedGoal() == SuperstructureGoal.STOW
//    }.onTrue(
//      robot.superstructureManager.requestGoal(SuperstructureGoal.CLIMB_BEFORE)
//        .alongWith(robot.climb.runClimbWheels())
//    )

    Trigger {
      driveController.hid.aButton &&
        !robot.intake.coralDetected() &&
        robot.superstructureManager.lastRequestedGoal() == SuperstructureGoal.STOW &&
        robot.superstructureManager.isAtPos()
    }.onTrue(
      Commands.sequence(
        robot.superstructureManager.requestGoal(SuperstructureGoal.CLIMB_BEFORE)
          .alongWith(robot.climb.runClimbWheels()),
        robot.climb.waitUntilCurrrentSpike(),
        WaitCommand(0.3276),
        Commands.parallel(
          robot.wrist.setPosition(WristConstants.CLIMB_DOWN.`in`(Radians)),
          robot.climb.stop(),
          robot.pivot.climbDown(),
          WaitUntilCommand { robot.pivot.climbReady() }
            .andThen(robot.elevator.climbDown())
        )
      )
    )
  }

  private fun scoreL2() {
    driveController.povLeft().onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.L2)
        .alongWith(robot.intake.holdCoral())
    )
  }

  private fun scoreL3() {
    driveController.povRight().onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.L3)
        .alongWith(robot.intake.holdCoral())
    )
  }

  private fun stopReefAlign() {
    mechanismController.start().onTrue(
      robot.driveCommand
    )
  }

  private fun stow() {
    driveController.povDown().onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
        .deadlineFor(robot.light.progressMaskGradient(percentageElevatorPosition))
        .alongWith(robot.intake.holdCoral())
        .alongWith(robot.climb.stop())
    )
  }

  private fun autoScoreLeftOuttake() {
    driveController.leftTrigger().onTrue(
      Commands.sequence(
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.LEFT))
          .deadlineFor(robot.light.gradient(MetersPerSecond.of(0.35), Color.kPurple, Color.kWhite)),
        Commands.parallel(
          robot.intake.outtakeCoral()
            .andThen(
              WaitUntilCommand { !robot.intake.coralDetected() && RobotBase.isReal() }
                .onlyIf { RobotBase.isReal() }
            )
            .andThen(WaitCommand(0.10))
            .andThen(robot.intake.stop())
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
            .andThen(
              WaitUntilCommand { !robot.intake.coralDetected() && RobotBase.isReal() }
                .onlyIf { RobotBase.isReal() }
            )
            .andThen(WaitCommand(0.10))
            .andThen(robot.intake.stop())
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

  private fun autoScoreLeft() {
    driveController.leftTrigger().onTrue(
      Commands.sequence(
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.LEFT))
          .deadlineFor(robot.light.gradient(MetersPerSecond.of(0.35), Color.kPurple, Color.kWhite)),
        robot.light.blink(Seconds.of(0.20), Color.kWhite)
          .withTimeout(1.5)
      )
    ).onFalse(
      robot.driveCommand
    )
  }

  private fun autoScoreRight() {
    driveController.rightTrigger().onTrue(
      Commands.sequence(
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.RIGHT))
          .deadlineFor(robot.light.gradient(MetersPerSecond.of(0.35), Color.kPurple, Color.kWhite)),
        robot.light.blink(Seconds.of(0.20), Color.kWhite)
          .withTimeout(1.5)
      )
    ).onFalse(
      robot.driveCommand
    )
  }

  private fun substationIntake() {
    driveController.leftBumper().onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
        .alongWith(robot.intake.intakeCoral())
        .andThen(WaitUntilCommand { robot.intake.coralDetected() && RobotBase.isReal() })
        .andThen(robot.intake.stop())
//        .deadlineFor(robot.light.gradient(MetersPerSecond.of(0.5), Color.kYellow, Color.kLightCoral, Color.kIndianRed))
        .andThen(
          robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
//            .alongWith(
//              robot.light.blink(Seconds.of(0.25), Color.kWhite)
//                .withTimeout(1.5)
//            )
        )
    )
  }

  private fun groundIntake() {
    driveController.leftBumper().onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.GROUND_INTAKE)
        .alongWith(robot.intake.intakeCoral())
        .andThen(WaitUntilCommand { robot.intake.coralDetected() && RobotBase.isReal() })
        .andThen(WaitCommand(0.25))
        .andThen(robot.intake.stop())
        .andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW))
        .andThen(robot.intake.holdCoral())
    )
  }

  private fun coralBlockSubstationIntake() {
    driveController.povDown().onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE_CORAL_IN_FRONT)
        .alongWith(robot.intake.intakeCoral())
        .andThen(WaitUntilCommand { robot.intake.coralDetected() && RobotBase.isReal() })
        .andThen(robot.intake.stop())
        .deadlineFor(robot.light.gradient(MetersPerSecond.of(0.5), Color.kYellow, Color.kLightCoral, Color.kIndianRed))
        .andThen(
          robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
            .alongWith(
              robot.light.blink(Seconds.of(0.25), Color.kWhite)
                .withTimeout(1.5)
            )
        )
    )
  }

  private fun outtake() {
    driveController.rightBumper().onTrue(
      ConditionalCommand(
        ConditionalCommand(
          robot.intake.outtakeL1(),
          ConditionalCommand(
            robot.intake.outtakeCoralPivot(),
            robot.intake.outtakeCoral()
          ) { robot.superstructureManager.requestedPivotSide() }
        ) { robot.superstructureManager.lastRequestedGoal() == SuperstructureGoal.L1 }
          .andThen(WaitUntilCommand { !robot.intake.coralDetected() })
          .andThen(WaitCommand(0.10))
          .andThen(robot.intake.stop())
          .andThen(robot.climb.stop())
          .andThen(
            robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
              .deadlineFor(robot.light.progressMaskGradient(percentageElevatorPosition))
          ),

        WaitCommand(0.15)
          .andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW))
      ) { RobotBase.isReal() }
    )
  }

  private fun score_l1() {
    Trigger { driveController.hid.aButton && robot.intake.coralDetected() }.onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.L1)
        .alongWith(robot.intake.holdCoral())
    )
  }

  private fun scoreDescore_l2() {
    driveController.x().onTrue(
      ConditionalCommand(
        ConditionalCommand(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L2_PIVOT),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L2)
        ) { robot.poseSubsystem.isPivotSide() }
          .alongWith(robot.intake.holdCoral()),
        robot.superstructureManager.requestGoal(SuperstructureGoal.L2_ALGAE_DESCORE)
          .alongWith(robot.intake.descoreAlgae())
      ) { robot.intake.coralDetected() }
    )
  }

  private fun scoreDescore_l3() {
    driveController.b().onTrue(
      ConditionalCommand(
        ConditionalCommand(
          robot.superstructureManager.requestGoal(SuperstructureGoal.L3_PIVOT),
          robot.superstructureManager.requestGoal(SuperstructureGoal.L3)
        ) { robot.poseSubsystem.isPivotSide() }
          .alongWith(robot.intake.holdCoral()),
        robot.superstructureManager.requestGoal(SuperstructureGoal.L3_ALGAE_DESCORE)
          .alongWith(robot.intake.descoreAlgae())
      ) { robot.intake.coralDetected() }
    )
  }

  private fun score_l4() {
    driveController.y().onTrue(
      ConditionalCommand(
        robot.superstructureManager.requestL4(SuperstructureGoal.L4_PIVOT),
        robot.superstructureManager.requestL4(SuperstructureGoal.L4)
      ) { robot.poseSubsystem.isPivotSide() }
        .alongWith(robot.intake.holdCoral())
    )
  }

  private fun testVoltagePivot() {
    characterizationController.rightTrigger().onTrue(
      robot.pivot.testVoltage()
    ).onFalse(
      robot.pivot.hold()
    )
  }

//  private fun runClimbWheels() {
//    characterizationController.leftTrigger().onTrue(
//      robot.climb.runClimbWheels()
//    ).onFalse(
//      robot.climb.stop()
//    )
//  }

  private fun wheelRadiusCharacterization() {
    characterizationController.leftTrigger().onTrue(
      WheelRadiusCharacterization(robot.drive, robot.poseSubsystem)
    )
  }

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

  private fun intakeCoral() {
    mechanismController.leftStick().onTrue(
      robot.intake.intakeCoral()
    ).toggleOnFalse(
      robot.intake.stop()
    )
  }

  private fun outtakeCoral() {
    mechanismController.rightStick().onTrue(
      robot.intake.outtakeCoralPivot()
    ).toggleOnFalse(
      robot.intake.stop()
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

  private fun resetGyro() {
    driveController.start().onTrue(
      ConditionalCommand(
        InstantCommand({ robot.poseSubsystem.heading = Rotation2d(PI) }),
        InstantCommand({ robot.poseSubsystem.heading = Rotation2d() })
      ) { DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red }
    )
  }

  private fun pointToRight() {
    driveController.a().onTrue(
      robot.driveCommand.pointAtAngleCommand(Rotation2d.fromDegrees(90.0))
    )
  }

  /** Characterization functions */
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
    nonRobotBindings()
    robotBindings()
    characterizationBindings()
  }
}
