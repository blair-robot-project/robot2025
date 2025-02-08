package frc.team449

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team449.commands.driveAlign.SimpleReefAlign
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveSim
import frc.team449.subsystems.superstructure.SuperstructureGoal
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI
import kotlin.random.Random

class ControllerBindings(
  private val driveController: CommandXboxController,
  private val mechanismController: CommandXboxController,
  private val robot: Robot
) {

  private fun robotBindings() {
    /** Call robot functions you create below */
    /** Driver: https://docs.google.com/drawings/d/13W3qlIxzIh5MTraZGWON7IqwJvovVr8eNBvjq8_vYZI/edit
     * Operator: https://docs.google.com/drawings/d/1lF4Roftk6932jMCQthgKfoJVPuTVSgnGZSHs5j68uo4/edit
     */
//    score_l1()
    score_l2()
    score_l3()
    score_l4()

    premove_l1()
    premove_l2()
    premove_l3()
    premove_l4()

    stow()

    mechanismController.pov(0).onTrue(
      robot.wrist.setPosition(PI/2)
    )
    mechanismController.pov(180).onTrue(
      robot.wrist.setPosition(0.0)
    )
  }

  private fun nonRobotBindings() {
    // slowDrive()

    if (RobotBase.isSimulation()) resetOdometrySim()

    resetGyro()
  }

  private fun stow() {
    driveController.button(6).onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
    )
  }

  private fun score_l1() {
    driveController.button(1).onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.L1)
        .alongWith(SimpleReefAlign(robot.drive, robot.poseSubsystem))
    )
  }

  private fun score_l2() {
    driveController.button(2).onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.L2)
        .alongWith(SimpleReefAlign(robot.drive, robot.poseSubsystem))
    )
  }

  private fun score_l3() {
    driveController.button(4).onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.L3)
        .alongWith(SimpleReefAlign(robot.drive, robot.poseSubsystem))
    )
  }

  private fun score_l4() {
    driveController.button(5).onTrue(
      robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
        .alongWith(SimpleReefAlign(robot.drive, robot.poseSubsystem))
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
  private fun characterizationExample() {
    /** Example
     *
     val exampleSubsystemRoutine = SysIdRoutine(
     SysIdRoutine.Config(
     Volts.of(0.5).per(Seconds.of(1.0)),
     Volts.of(3.0),
     Seconds.of(10.0)
     ) { state -> SignalLogger.writeString("state", state.toString()) },
     Mechanism(
     { voltage: Measure<Voltage> ->
     run { robot.shooter.setVoltage(voltage.`in`(Volts)) }
     },
     null,
     robot.shooter,
     "shooter"
     )
     )

     // Quasistatic Forwards
     driveController.povUp().onTrue(
     exampleSubsystemRoutine.quasistatic(SysIdRoutine.Direction.kForward)
     )

     // Quasistatic Reverse
     driveController.povDown().onTrue(
     exampleSubsystemRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
     )

     // Dynamic Forwards
     driveController.povRight().onTrue(
     exampleSubsystemRoutine.dynamic(SysIdRoutine.Direction.kForward)
     )

     // Dynamic Reverse
     driveController.povLeft().onTrue(
     exampleSubsystemRoutine.dynamic(SysIdRoutine.Direction.kReverse)
     )
     */
  }

  private fun elevatorCharacterizaton() {
    val elevatorSubsystemRoutine = SysIdRoutine(
      SysIdRoutine.Config(
        Volts.of(0.5).per(Second),
        Volts.of(3.0),
        Seconds.of(10.0)
      ) { state -> SignalLogger.writeString("state", state.toString()) },
      SysIdRoutine.Mechanism(
        { voltage: Voltage ->
          run { robot.elevator.setVoltage(voltage.`in`(Volts)) }
        },
        null,
        robot.elevator,
        "elevator"
      )
    )

    driveController.povUp().onTrue(
      elevatorSubsystemRoutine.quasistatic(SysIdRoutine.Direction.kForward)
    )

    driveController.povDown().onTrue(
      elevatorSubsystemRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
    )

    driveController.povRight().onTrue(
      elevatorSubsystemRoutine.dynamic(SysIdRoutine.Direction.kForward)
    )

    driveController.povLeft().onTrue(
      elevatorSubsystemRoutine.dynamic(SysIdRoutine.Direction.kReverse)
    )
  }

  fun pivotCharacterizaton() {
    val pivotSubsystemRoutine = SysIdRoutine(
      SysIdRoutine.Config(
        Volts.of(0.5).per(Second),
        Volts.of(3.0),
        Seconds.of(10.0)
      ) { state -> SignalLogger.writeString("state", state.toString()) },
      SysIdRoutine.Mechanism(
        { voltage: Voltage ->
          run { robot.pivot.setVoltage(voltage.`in`(Volts)) }
        },
        null,
        robot.pivot,
        "elevator"
      )
    )

    driveController.povUp().onTrue(
      pivotSubsystemRoutine.quasistatic(SysIdRoutine.Direction.kForward)
    )

    driveController.povDown().onTrue(
      pivotSubsystemRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
    )

    driveController.povRight().onTrue(
      pivotSubsystemRoutine.dynamic(SysIdRoutine.Direction.kForward)
    )

    driveController.povLeft().onTrue(
      pivotSubsystemRoutine.dynamic(SysIdRoutine.Direction.kReverse)
    )
  }

  /** Try not to touch, just add things to the robot or nonrobot bindings */
  fun bindButtons() {
    nonRobotBindings()
    robotBindings()
  }
}
