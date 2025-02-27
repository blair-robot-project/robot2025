package frc.team449

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
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

  var STOW_AFTER_AUTOSCORE = false

  private fun robotBindings() {
    /** Call robot functions you create below */
    /** Driver: https://docs.google.com/drawings/d/13W3qlIxzIh5MTraZGWON7IqwJvovVr8eNBvjq8_vYZI/edit
     * Operator: https://docs.google.com/drawings/d/1lF4Roftk6932jMCQthgKfoJVPuTVSgnGZSHs5j68uo4/edit
     */
    score_l1()
    score_l2()
    score_l3()
    score_l4()

    autoScoreLeft()
    autoScoreRight()
//    autoScoreStowTrigger()

    substationIntake()
    coralOuttake()

//    premove_l1()
//    premove_l2()
//    premove_l3()
//    premove_l4()

    stow()
    climb()
    algaeDescoreL2()
  }

  private fun characterizationBindings() {
    manualElevator()
    manualPivot()
    manualWrist()
    testVoltagePivot()
    runClimbWheels()

    pivotCharacterizaton()
  }

  private fun nonRobotBindings() {
    // slowDrive()

    /** NOTE: If you want to see simulated vision convergence times with this function, go to simulationPeriodic in
     * RobotBase and change the passed in pose to it.simulationPeriodic to robot.drive.odometryPose
     */
//    if (RobotBase.isSimulation()) resetOdometrySim()

    resetGyro()
  }

  private fun climb() {
    mechanismController.rightBumper().onTrue(
      robot.climb.runClimbWheels()
        .andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.CLIMB))
        .andThen(WaitUntilCommand { robot.climb.isClimbEngaged() })
        .andThen(WaitCommand(0.15))
        .andThen(robot.pivot.climbDown())
    )
  }

  private fun algaeDescoreL2() {
    mechanismController.x().onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.L2_ALGAE_DESCORE)
    )
  }

  private fun stow() {
    mechanismController.a().onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
    )
  }

  private fun autoScoreLeft() {
    driveController.leftTrigger().onTrue(
      SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.LEFT))
//        .andThen(
//          WaitUntilCommand {
//            (
//              robot.superstructureManager.lastRequestedGoal() == SuperstructureGoal.L1 ||
//                robot.superstructureManager.lastRequestedGoal() == SuperstructureGoal.L2 ||
//                robot.superstructureManager.lastRequestedGoal() == SuperstructureGoal.L3 ||
//                robot.superstructureManager.lastRequestedGoal() == SuperstructureGoal.L4
//              ) &&
//              robot.superstructureManager.isAtPos()
//          }
//        )
//        .andThen(PrintCommand("boutta outtake this coral"))
//        .andThen(robot.intake.outtakeCoral())
//        .andThen(WaitUntilCommand { !robot.intake.coralDetected() && RobotBase.isReal() })
//        .andThen(WaitCommand(0.25))
//        .andThen(robot.intake.stop())
//        .andThen(InstantCommand({ STOW_AFTER_AUTOSCORE = true }))
    )
  }

  private fun autoScoreStowTrigger() {
    Trigger { STOW_AFTER_AUTOSCORE }.onTrue(
      InstantCommand({ STOW_AFTER_AUTOSCORE = false })
        .andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW))
    )
  }

  private fun autoScoreRight() {
    driveController.rightTrigger().onTrue(
      SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(FieldConstants.ReefSide.RIGHT))
//        .andThen(
//          WaitUntilCommand {
//            (
//              robot.superstructureManager.lastRequestedGoal() == SuperstructureGoal.L1 ||
//                robot.superstructureManager.lastRequestedGoal() == SuperstructureGoal.L2 ||
//                robot.superstructureManager.lastRequestedGoal() == SuperstructureGoal.L3 ||
//                robot.superstructureManager.lastRequestedGoal() == SuperstructureGoal.L4
//              ) &&
//              robot.superstructureManager.isAtPos()
//          }
//        )
//        .andThen(PrintCommand("boutta outtake this coral"))
//        .andThen(robot.intake.outtakeCoral())
//        .andThen(WaitUntilCommand { !robot.intake.coralDetected() && RobotBase.isReal() })
//        .andThen(WaitCommand(0.25))
//        .andThen(robot.intake.stop())
//        .andThen(InstantCommand({ STOW_AFTER_AUTOSCORE = true }))
    )
  }

  private fun substationIntake() {
    driveController.leftBumper().onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
        .alongWith(robot.intake.intakeCoral())
        .andThen(WaitUntilCommand { robot.intake.coralDetected() && RobotBase.isReal() })
        .andThen(robot.intake.holdCoral())
        .andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW))
    )
  }

  private fun coralOuttake() {
    driveController.rightBumper().onTrue(
      ConditionalCommand(
        robot.intake.outtakeCoral()
          .andThen(WaitUntilCommand { !robot.intake.coralDetected() && RobotBase.isReal() })
          .andThen(WaitCommand(0.10))
          .andThen(robot.intake.stop())
          .andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)),
        WaitCommand(0.15)
          .andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW))
      ) { RobotBase.isReal() }
    )
  }

  private fun score_l1() {
    driveController.povDown().onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.L1)
    )
  }

  private fun score_l2() {
    driveController.x().onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.L2)
    )
  }

  private fun score_l3() {
    driveController.b().onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.L3)
    )
  }

  private fun score_l4() {
    driveController.y().onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
    )
  }

  private fun premove_l1() {
    mechanismController.a().onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.L1_PREMOVE)
    )
  }

  private fun premove_l2() {
    mechanismController.x().onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.L2_PREMOVE)
    )
  }

  private fun premove_l3() {
    mechanismController.b().onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.L3_PREMOVE)
    )
  }

  private fun premove_l4() {
    mechanismController.y().onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)
    )
  }

  private fun testVoltagePivot() {
    characterizationController.rightTrigger().onTrue(
      robot.pivot.testVoltage()
    ).onFalse(
      robot.pivot.hold()
    )
  }

  private fun runClimbWheels() {
    characterizationController.leftTrigger().onTrue(
      robot.climb.runClimbWheels()
    ).onFalse(
      robot.climb.stop()
    )
  }

  private fun wheelRadiusCharacterization() {
    characterizationController.leftTrigger().onTrue(
      WheelRadiusCharacterization(robot.drive, robot.poseSubsystem)
    )
  }

  private fun manualPivot() {
    // up
    characterizationController.b().onTrue(
      robot.pivot.manualUp()
    ).onFalse(robot.pivot.hold())
    // down
    characterizationController.x().onTrue(
      robot.pivot.manualDown()
    ).onFalse(robot.pivot.hold())
  }

  private fun manualElevator() {
    // up
    characterizationController.y().onTrue(
      robot.elevator.manualUp()
    ).onFalse(robot.elevator.hold())

    // down
    characterizationController.a().onTrue(
      robot.elevator.manualDown()
    ).onFalse(robot.elevator.hold())
  }

  private fun manualWrist() {
    // up
    characterizationController.rightBumper().onTrue(
      robot.wrist.manualUp()
    ).onFalse(robot.wrist.hold())

    // down
    characterizationController.leftBumper().onTrue(
      robot.wrist.manualDown()
    ).onFalse(robot.wrist.hold())
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
