package frc.team449.subsystems.superstructure
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.util.function.BooleanConsumer
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.Robot
import frc.team449.subsystems.drive.swerve.SwerveModuleKraken
import java.util.function.DoubleSupplier
import java.util.function.BooleanSupplier
import java.util.function.DoubleConsumer
import kotlin.math.abs

class BuiltInTests (private val robot: Robot) {
  private val drive = robot.drive
  private val intake = robot.intake
  private val pivot = robot.pivot
  private val elevator = robot.elevator
  private val wrist = robot.wrist
  private val manager = robot.superstructureManager

  private var modulesAtSetpoint = false
  private var autotestStart = ""
  private val timer = Timer()
  var userInput = false
  var runningTest = false

  fun getScoringTests(): Command {
    return Commands.sequence(
      InstantCommand({
        userInput = false
      }),
      PrintCommand("Running scoring position tests. Press d-pad at any time to cancel."),
      manager.requestGoal(SuperstructureGoal.L2),
      WaitCommand(BITConstants.EXTERNAL_WAIT),
      manager.requestGoal(SuperstructureGoal.L4),
      WaitCommand(BITConstants.EXTERNAL_WAIT),
      manager.requestGoal(SuperstructureGoal.L1),
      WaitCommand(BITConstants.EXTERNAL_WAIT),
      manager.requestGoal(SuperstructureGoal.L3),
      WaitCommand(BITConstants.EXTERNAL_WAIT),
      manager.requestGoal(SuperstructureGoal.STOW)
    ).onlyWhile { !userInput }.finallyDo(Runnable {
      if (userInput) {
        println("Scoring Position Tests canceled.")
      } else {
        println("Scoring Position Tests done.")
      }
    })
  }

  private fun waitUntilDriveAtTolerance(speeds: ChassisSpeeds): Command {
    return InstantCommand({
      robot.drive.set(speeds)
      modulesAtSetpoint = false
    }).andThen(InstantCommand({
      var atSetpoint = true
      robot.drive.modules.forEach {
        val krakenModule = it as? SwerveModuleKraken //we use krakens
        val angleDistance = abs(krakenModule?.turnController?.setpoint!! - krakenModule.state.angle.radians)
        if(angleDistance >= BITConstants.DRIVE_ANGLE_TOLERANCE) {
          atSetpoint = false
        }
      }
      modulesAtSetpoint = atSetpoint
    }).until { modulesAtSetpoint })
  }

  fun getDriveTests(): Command {
    return Commands.sequence(
      InstantCommand({ userInput = false }),
      PrintCommand("Running drive tests. Make sure the wheels are in the air. Press d-pad at any time to cancel."),
      waitUntilDriveAtTolerance(ChassisSpeeds(2.0, 0.0, 0.0)),
      WaitCommand(BITConstants.DRIVE_WAIT),
      waitUntilDriveAtTolerance(ChassisSpeeds(0.0, 2.0, 0.0)),
      WaitCommand(BITConstants.DRIVE_WAIT),
      waitUntilDriveAtTolerance(ChassisSpeeds(1.0, 1.0, 0.0)),
      WaitCommand(BITConstants.DRIVE_WAIT),
      waitUntilDriveAtTolerance(ChassisSpeeds(-1.0, -1.0, 0.0)),
      WaitCommand(BITConstants.DRIVE_WAIT),
    ).onlyWhile { !userInput }.finallyDo(Runnable {
      if (userInput) {
        println("Drive Tests canceled.")
      } else {
        println("Drive Tests done.")
      }
    })
  }

  fun getIntakeTests(): Command {
    return Commands.sequence(
      InstantCommand({ userInput = false }),
      PrintCommand("Running intake tests. Grab a coral. Press d-pad at any time to cancel."),
      WaitCommand(0.5),
      intake.intakeCoral(),
      PrintCommand("Please feed the robot a coral."),
      WaitUntilCommand {intake.coralDetected()},
      WaitCommand(1.0),
      intake.stop()
    ).onlyWhile { !userInput }.finallyDo(Runnable {
      if (userInput) {
        println("Intake Tests canceled.")
      } else {
        println("Intake Tests done.")
      }
    })
  }

  private fun checkVoltageWait(boolSupplier: BooleanSupplier, dblSupplier: DoubleSupplier, highVoltageVal: Double): Command {
    return FunctionalCommand(
      {}, {
        if(dblSupplier.asDouble > highVoltageVal) {
          println("Voltage is High! Currently Reading: " + dblSupplier.asDouble)
        } }, {},
      boolSupplier
    )
  }

  private fun runPositionTest(name: String, setpoint: Double, slowDeadline: Double, realDeadline: Double, setpointName: String): Command {
    return Commands.sequence(
      InstantCommand({
        timer.restart()
        autotestStart = when (name) {
          "pivot" -> "${Units.radiansToDegrees(pivot.positionSupplier.get())} degrees"
          "elevator" -> "${Units.radiansToDegrees(elevator.positionSupplier.get())} meters"
          else -> "${Units.radiansToDegrees(wrist.positionSupplier.get())} degrees"
        }
      }),
      when (name) {
        "pivot" -> pivot.setPosition(setpoint)
        "elevator" -> elevator.setPosition(setpoint)
        else -> wrist.setPosition(setpoint)
      },
      when (name) {
        "pivot" -> checkVoltageWait({ pivot.atSetpoint(BITConstants.PIVOT_TOLERANCE) }, {pivot.getMotorVoltage()}, BITConstants.HIGH_PIVOT_VOLTAGE)
        "elevator" -> checkVoltageWait({ elevator.atSetpoint(BITConstants.ELEVATOR_TOLERANCE) }, {elevator.getMotorVoltage()}, BITConstants.HIGH_ELEVATOR_VOLTAGE)
        else -> checkVoltageWait({ wrist.atSetpoint(BITConstants.WRIST_TOLERANCE) }, {wrist.getMotorVoltage()}, BITConstants.HIGH_WRIST_VOLTAGE)
      }
        .raceWith(WaitUntilCommand{timer.get() > realDeadline})
        .finallyDo(Runnable {
          if (timer.get() > slowDeadline) {
            if (timer.get() > realDeadline) {
              println("$name is crazy slow, taking ${timer.get()} seconds to get to $setpointName from $autotestStart")
            } else {
              println("$name is slow, taking ${timer.get()} seconds to get to $setpointName from $autotestStart")
            }
          } else {
            println("$name is good getting to $setpointName from $autotestStart")
          }
        }),
      WaitCommand(when (name) {
        "pivot" -> BITConstants.PIVOT_WAIT
        "elevator" -> BITConstants.ELEVATOR_WAIT
        else -> BITConstants.WRIST_WAIT
      })
    )
  }

  fun getPositionTests(): Command {
    val pivotTests = SequentialCommandGroup(
      InstantCommand({ userInput = false }),
      PrintCommand("Running individual position tests. Press d-pad at any time to cancel.")
    )
    listOf(
      BITConstants.PIVOT_SETPOINT_ONE,
      BITConstants.PIVOT_SETPOINT_TWO,
      BITConstants.PIVOT_SETPOINT_THREE,
      BITConstants.PIVOT_SETPOINT_FOUR,
      BITConstants.PIVOT_SETPOINT_FIVE,
      BITConstants.PIVOT_SETPOINT_SIX
    ).forEach { pivotTests.addCommands(runPositionTest("pivot", it, BITConstants.PIVOT_EXPECTED_TIME,
      BITConstants.PIVOT_TIMEOUT, "the angle ${Units.radiansToDegrees(it)} in degrees")) }

    val elevatorTests = SequentialCommandGroup()
    listOf(
      BITConstants.ELEVATOR_SETPOINT_ONE,
      BITConstants.ELEVATOR_SETPOINT_TWO,
      BITConstants.ELEVATOR_SETPOINT_THREE,
      BITConstants.ELEVATOR_SETPOINT_FOUR
    ).forEach { pivotTests.addCommands(runPositionTest("elevator", it, BITConstants.ELEVATOR_EXPECTED_TIME,
      BITConstants.ELEVATOR_TIMEOUT, "the height $it in meters")) }

    val wristTests = SequentialCommandGroup()
    listOf(
      BITConstants.WRIST_SETPOINT_ONE,
      BITConstants.WRIST_SETPOINT_TWO,
      BITConstants.WRIST_SETPOINT_THREE,
      BITConstants.WRIST_SETPOINT_FOUR,
      BITConstants.WRIST_SETPOINT_FIVE
    ).forEach { pivotTests.addCommands(runPositionTest("wrist", it, BITConstants.WRIST_EXPECTED_TIME,
      BITConstants.WRIST_TIMEOUT, "the angle ${Units.radiansToDegrees(it)} in degrees")) }

    pivotTests.addCommands(
      WaitCommand(BITConstants.EXTERNAL_WAIT),
      elevatorTests,
      WaitCommand(BITConstants.EXTERNAL_WAIT),
      wristTests,
      WaitCommand(BITConstants.EXTERNAL_WAIT),
      manager.requestGoal(SuperstructureGoal.STOW)
    )
    return pivotTests.onlyWhile { !userInput }.finallyDo(Runnable {
      if (userInput) {
        println("Individual Position Tests canceled.")
      } else {
        println("Individual Position Tests done.")
      }
    })
  }

  private fun getROMTest(
    voltage: Double,
    voltageSetter: DoubleConsumer,
    atFrontSupplier: BooleanSupplier,
    atBackSupplier: BooleanSupplier,
    stopCommand: Command
  ): Command {
    return SequentialCommandGroup(
        InstantCommand({ voltageSetter.accept(voltage) }),
        WaitUntilCommand(atFrontSupplier),
        WaitCommand(0.25),
        InstantCommand({ voltageSetter.accept(-voltage) }),
        WaitUntilCommand(atBackSupplier),
        stopCommand,
        WaitCommand(0.25)
    )
  }


  fun getROMTests(): Command {
    val cmd = SequentialCommandGroup(
      InstantCommand({ userInput = false }),
      PrintCommand("Running Range of Motion Tests. Press d-pad at any time to cancel.")
    )
    listOf(
      BITConstants.PIVOT_SLOW_VOLTAGE,
      BITConstants.PIVOT_MEDIUM_VOLTAGE,
      BITConstants.PIVOT_FAST_VOLTAGE,
    ).forEach { vltg ->
      cmd.addCommands(
        getROMTest(
          vltg,
          { pivot.setVoltageChar(it) },
          {pivot.atSetpoint(BITConstants.PIVOT_HARDSTOP_FRONT)},
          {pivot.atSetpoint(BITConstants.PIVOT_HARDSTOP_BACK)},
          pivot.stop()
        )
      )
    }
    cmd.addCommands(pivot.setPosition(BITConstants.PIVOT_SETPOINT_SIX), WaitUntilCommand{pivot.atSetpoint()})

    listOf(
      BITConstants.ELEVATOR_SLOW_VOLTAGE,
      BITConstants.ELEVATOR_MEDIUM_VOLTAGE,
      BITConstants.ELEVATOR_FAST_VOLTAGE,
    ).forEach { vltg ->
      cmd.addCommands(
        getROMTest(
          vltg,
          {elevator.setVoltage(it)},
          {elevator.atSetpoint(BITConstants.ELEVATOR_HARDSTOP_TOP)},
          {elevator.atSetpoint(BITConstants.ELEVATOR_HARDSTOP_BOTTOM)},
          elevator.stop()
        )
      )
    }

    listOf(
      BITConstants.WRIST_SLOW_VOLTAGE,
      BITConstants.WRIST_MEDIUM_VOLTAGE,
      BITConstants.WRIST_FAST_VOLTAGE,
    ).forEach { vltg ->
      cmd.addCommands(
        getROMTest(
          vltg,
          {wrist.setVoltageChar(it)},
          {wrist.atSetpoint(BITConstants.WRIST_HARDSTOP_FRONT)},
          {wrist.atSetpoint(BITConstants.WRIST_HARDSTOP_BACK)},
          wrist.stop()
        )
      )
    }
    return cmd.onlyWhile { !userInput }.finallyDo(Runnable {
      if (userInput) {
        println("ROM Tests canceled.")
      } else {
        println("ROM Tests done.")
      }
    })
  }

}