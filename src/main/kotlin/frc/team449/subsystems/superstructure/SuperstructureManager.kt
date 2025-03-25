package frc.team449.subsystems.superstructure

import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.Robot
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.superstructure.elevator.Elevator
import frc.team449.subsystems.superstructure.elevator.ElevatorConstants
import frc.team449.subsystems.superstructure.pivot.Pivot
import frc.team449.subsystems.superstructure.pivot.PivotConstants
import frc.team449.subsystems.superstructure.wrist.Wrist
import frc.team449.subsystems.superstructure.wrist.WristConstants
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier

class SuperstructureManager(
  private val elevator: Elevator,
  private val pivot: Pivot,
  private val wrist: Wrist,
  private val drive: SwerveDrive
) {

  private var lastGoal = SuperstructureGoal.STOW
  private var ready = false
  //auto test vars
  private var autotestStart = ""
  private val timer = Timer()
  private lateinit var autotestCommand: Command

  fun requestGoal(goal: SuperstructureGoal.SuperstructureState): Command {
    return InstantCommand({ SuperstructureGoal.applyDriveDynamics(drive, goal.driveDynamics) })
      .andThen(InstantCommand({ ready = false }))
      .andThen(InstantCommand({ lastGoal = goal }))
      .andThen(
        ConditionalCommand(
          // if extending
          Commands.sequence(
            InstantCommand({ SuperstructureGoal.applyDriveDynamics(drive, goal.driveDynamics) }),
            Commands.parallel(
              wrist.setPosition(goal.wrist.`in`(Radians)),
              pivot.setPosition(goal.pivot.`in`(Radians))
            ),
            WaitUntilCommand { wrist.elevatorReady() },
            elevator.setPosition(goal.elevator.`in`(Meters)),
//            WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() && elevator.atSetpoint() },
            WaitUntilCommand { wrist.atSetpoint() || pivot.atSetpoint() },
            pivot.hold().onlyIf { pivot.atSetpoint() },
            wrist.hold().onlyIf { wrist.atSetpoint() },
            WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() },
            pivot.hold(),
            wrist.hold(),
            WaitUntilCommand { elevator.atSetpoint() },
            holdAll()
          ),

          // if retracting
          Commands.sequence(
            elevator.setPosition(goal.elevator.`in`(Meters)),
            wrist.hold(),
            wrist.setPosition(WristConstants.ELEVATOR_READY.`in`(Radians))
              .onlyIf { goal.wrist > WristConstants.ELEVATOR_READY },
            WaitUntilCommand { elevator.atSetpoint() },
            Commands.parallel(
              pivot.setPosition(goal.pivot.`in`(Radians)),
              wrist.setPosition(goal.wrist.`in`(Radians))
            ),
            WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() },
//            elevator.hold()
//              .repeatedly()
//              .until { wrist.atSetpoint() && pivot.atSetpoint() },
            InstantCommand({ SuperstructureGoal.applyDriveDynamics(drive, goal.driveDynamics) }),
            holdAll()
          )
        ) { goal.elevator.`in`(Meters) >= elevator.positionSupplier.get() }
      )
      .andThen(InstantCommand({ ready = true }))
  }

  fun requestL4(goal: SuperstructureGoal.SuperstructureState = SuperstructureGoal.L4): Command {
    return InstantCommand({ SuperstructureGoal.applyDriveDynamics(drive, goal.driveDynamics) })
      .andThen(InstantCommand({ ready = false }))
      .andThen(InstantCommand({ lastGoal = goal }))
      .andThen(
        ConditionalCommand(
          // if extending
          Commands.sequence(
            InstantCommand({ SuperstructureGoal.applyDriveDynamics(drive, goal.driveDynamics) }),
            Commands.parallel(
              wrist.setPosition(goal.wrist.`in`(Radians)),
              pivot.setPosition(goal.pivot.`in`(Radians))
            ),
            WaitUntilCommand { wrist.elevatorReady() },
            elevator.setPositionCarriage(goal.elevator.`in`(Meters)),
//            WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() && elevator.atSetpoint() },
            WaitUntilCommand { wrist.atSetpoint() || pivot.atSetpoint() },
            pivot.hold().onlyIf { pivot.atSetpoint() },
            wrist.hold().onlyIf { wrist.atSetpoint() },
            WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() },
            pivot.hold(),
            wrist.hold(),
            WaitUntilCommand { elevator.atSetpoint() },
            Commands.parallel(
              pivot.hold(),
              wrist.hold(),
              elevator.holdCarriage()
            )
          ),

          // if retracting
          Commands.sequence(
            elevator.setPositionCarriage(goal.elevator.`in`(Meters)),
            wrist.hold(),
            wrist.setPosition(WristConstants.ELEVATOR_READY.`in`(Radians))
              .onlyIf { goal.wrist > WristConstants.ELEVATOR_READY },
            WaitUntilCommand { elevator.atSetpoint() },
            Commands.parallel(
              pivot.setPosition(goal.pivot.`in`(Radians)),
              wrist.setPosition(goal.wrist.`in`(Radians))
            ),
            WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() },
            InstantCommand({ SuperstructureGoal.applyDriveDynamics(drive, goal.driveDynamics) }),
            Commands.parallel(
              pivot.hold(),
              wrist.hold(),
              elevator.holdCarriage()
            )
          )
        ) { goal.elevator.`in`(Meters) >= elevator.positionSupplier.get() }
      )
      .andThen(InstantCommand({ ready = true }))
  }

  fun isAtPos(): Boolean {
    return ready
  }

  fun lastRequestedGoal(): SuperstructureGoal.SuperstructureState {
    return lastGoal
  }

  fun holdAll(): Command {
    return Commands.parallel(
      pivot.hold(),
      wrist.hold(),
      elevator.hold()
    )
  }

  private fun checkVoltageWait(boolSupplier: BooleanSupplier, dblSupplier: DoubleSupplier, highVoltageVal: Double): Command {
    return InstantCommand({ if(dblSupplier.asDouble > highVoltageVal)
      println("Voltage is High! Currently Reading: " + dblSupplier.asDouble) }).until(boolSupplier)
  }

  private fun runTest(name: String, setpoint: Double, slowDeadline: Double, realDeadline: Double, setpointName: String): Command {
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
        "pivot" -> checkVoltageWait({ pivot.atSetpoint(AutoTestConstants.PIVOT_TOLERANCE) }, {pivot.getMotorVoltage()}, AutoTestConstants.HIGH_PIVOT_VOLTAGE)
        "elevator" -> checkVoltageWait({ elevator.atSetpoint(AutoTestConstants.ELEVATOR_TOLERANCE) }, {elevator.getMotorVoltage()}, AutoTestConstants.HIGH_ELEVATOR_VOLTAGE)
        else -> checkVoltageWait({ wrist.atSetpoint(AutoTestConstants.WRIST_TOLERANCE) }, {wrist.getMotorVoltage()}, AutoTestConstants.HIGH_WRIST_VOLTAGE)
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
        "pivot" -> AutoTestConstants.PIVOT_WAIT
        "elevator" -> AutoTestConstants.ELEVATOR_WAIT
        else -> AutoTestConstants.WRIST_WAIT
      })
    )
  }

  private fun generatePremoveTests(): List<Command> {
    val pivotTests = SequentialCommandGroup()
    listOf(
      AutoTestConstants.PIVOT_SETPOINT_ONE,
      AutoTestConstants.PIVOT_SETPOINT_TWO,
      AutoTestConstants.PIVOT_SETPOINT_THREE,
      AutoTestConstants.PIVOT_SETPOINT_FOUR,
      AutoTestConstants.PIVOT_SETPOINT_FIVE,
      AutoTestConstants.PIVOT_SETPOINT_SIX
    ).forEach { pivotTests.addCommands(runTest("pivot", it, AutoTestConstants.PIVOT_EXPECTED_TIME,
      AutoTestConstants.PIVOT_TIMEOUT, "the angle ${Units.radiansToDegrees(it)} in degrees")) }

    val elevatorTests = SequentialCommandGroup()
    listOf(
      AutoTestConstants.ELEVATOR_SETPOINT_ONE,
      AutoTestConstants.ELEVATOR_SETPOINT_TWO,
      AutoTestConstants.ELEVATOR_SETPOINT_THREE,
      AutoTestConstants.ELEVATOR_SETPOINT_FOUR
    ).forEach { pivotTests.addCommands(runTest("elevator", it, AutoTestConstants.ELEVATOR_EXPECTED_TIME,
      AutoTestConstants.ELEVATOR_TIMEOUT, "the height $it in meters")) }

    val wristTests = SequentialCommandGroup()
    listOf(
      AutoTestConstants.WRIST_SETPOINT_ONE,
      AutoTestConstants.WRIST_SETPOINT_TWO,
      AutoTestConstants.WRIST_SETPOINT_THREE,
      AutoTestConstants.WRIST_SETPOINT_FOUR,
      AutoTestConstants.WRIST_SETPOINT_FIVE
    ).forEach { pivotTests.addCommands(runTest("wrist", it, AutoTestConstants.WRIST_EXPECTED_TIME,
      AutoTestConstants.WRIST_TIMEOUT, "the angle ${Units.radiansToDegrees(it)} in degrees")) }

    return listOf(
      pivotTests,
      WaitCommand(AutoTestConstants.EXTERNAL_WAIT),
      elevatorTests,
      WaitCommand(AutoTestConstants.EXTERNAL_WAIT),
      wristTests,
      WaitCommand(AutoTestConstants.EXTERNAL_WAIT),
      requestGoal(SuperstructureGoal.STOW)
      )
  }

  private fun doPivotROM(cruiseVel: AngularVelocity, maxAccel: AngularAcceleration): Command {
    return Commands.sequence(
      InstantCommand({
        PivotConstants.changeMaxAccel(maxAccel)
        PivotConstants.changeCruiseVel(cruiseVel)
      }),
      pivot.setPosition(AutoTestConstants.PIVOT_HARDSTOP_FRONT - Units.degreesToRadians(1.0)),
      checkVoltageWait({ pivot.atSetpoint(AutoTestConstants.PIVOT_TOLERANCE) }, {pivot.getMotorVoltage()}, AutoTestConstants.HIGH_PIVOT_VOLTAGE),
      pivot.setPosition(AutoTestConstants.PIVOT_HARDSTOP_BACK + Units.degreesToRadians(1.0)),
      checkVoltageWait({ pivot.atSetpoint(AutoTestConstants.PIVOT_TOLERANCE) }, {pivot.getMotorVoltage()}, AutoTestConstants.HIGH_PIVOT_VOLTAGE)
    )
  }

  private fun doElevatorROM(cruiseVel: Double, maxAccel: Double): Command {
    return Commands.sequence(
      InstantCommand({
        ElevatorConstants.changeMaxAccel(maxAccel)
        ElevatorConstants.changeCruiseVel(cruiseVel)
      }),
      elevator.setPosition(SuperstructureGoal.L4.elevator.`in`(Meters) + Units.inchesToMeters(2.0)),
      checkVoltageWait({ elevator.atSetpoint(AutoTestConstants.ELEVATOR_TOLERANCE) }, {elevator.getMotorVoltage()}, AutoTestConstants.HIGH_ELEVATOR_VOLTAGE),
      elevator.setPosition(SuperstructureGoal.L1.elevator.`in`(Meters)),
      checkVoltageWait({ elevator.atSetpoint(AutoTestConstants.ELEVATOR_TOLERANCE) }, {elevator.getMotorVoltage()}, AutoTestConstants.HIGH_ELEVATOR_VOLTAGE),
    )
  }

  private fun doWristROM(cruiseVel: AngularVelocity, maxAccel: AngularAcceleration): Command {
    return Commands.sequence(
      InstantCommand({
        WristConstants.changeMaxAccel(maxAccel)
        WristConstants.changeCruiseVel(cruiseVel)
      }),
      wrist.setPosition(SuperstructureGoal.L4.elevator.`in`(Meters) + Units.inchesToMeters(2.0)),
      checkVoltageWait({ wrist.atSetpoint(AutoTestConstants.WRIST_TOLERANCE) }, {wrist.getMotorVoltage()}, AutoTestConstants.HIGH_WRIST_VOLTAGE),
      wrist.setPosition(SuperstructureGoal.L1.elevator.`in`(Meters)),
      checkVoltageWait({ wrist.atSetpoint(AutoTestConstants.WRIST_TOLERANCE) }, {wrist.getMotorVoltage()}, AutoTestConstants.HIGH_WRIST_VOLTAGE),
    )
  }

  private fun generateROMTests(): List<Command> {
    val pivotRomTest = Commands.sequence(
      doPivotROM(PivotConstants.CRUISE_VEL_VALUE / 1.6, PivotConstants.MAX_ACCEL_VALUE / 1.6),
      doPivotROM(PivotConstants.CRUISE_VEL_VALUE / 1.33, PivotConstants.MAX_ACCEL_VALUE / 1.33),
      doPivotROM(PivotConstants.CRUISE_VEL_VALUE, PivotConstants.MAX_ACCEL_VALUE),
      pivot.setPosition(AutoTestConstants.PIVOT_SETPOINT_SIX),
      WaitUntilCommand { pivot.atSetpoint() },
    )

    val elevatorRomTest = Commands.sequence(
      doElevatorROM(ElevatorConstants.CRUISE_VEL_VALUE / 1.6, ElevatorConstants.MAX_ACCEL_VALUE / 1.6),
      doElevatorROM(ElevatorConstants.CRUISE_VEL_VALUE / 1.33, ElevatorConstants.MAX_ACCEL_VALUE / 1.33),
      doElevatorROM(ElevatorConstants.CRUISE_VEL_VALUE, ElevatorConstants.MAX_ACCEL_VALUE),
      requestGoal(SuperstructureGoal.STOW)
    )

    val wristRomTest = Commands.sequence(
      doWristROM(WristConstants.CRUISE_VEL_VALUE / 1.6, WristConstants.MAX_ACCEL_VALUE / 1.6),
      doWristROM(WristConstants.CRUISE_VEL_VALUE / 1.33, WristConstants.MAX_ACCEL_VALUE / 1.33),
      doWristROM(WristConstants.CRUISE_VEL_VALUE, WristConstants.MAX_ACCEL_VALUE),
      requestGoal(SuperstructureGoal.STOW)
    )
      
    return listOf(
      pivotRomTest,
      WaitCommand(AutoTestConstants.EXTERNAL_WAIT),
      elevatorRomTest,
      WaitCommand(AutoTestConstants.EXTERNAL_WAIT),
      wristRomTest,
      WaitCommand(AutoTestConstants.EXTERNAL_WAIT),
      requestGoal(SuperstructureGoal.STOW),
      WaitCommand(AutoTestConstants.EXTERNAL_WAIT)
      )
    
  }

  private fun getUserConfirmation(scanner: java.util.Scanner, timeout: Double): Boolean {
    var input: String
    timer.restart()
    while(timer.get() < timeout) {
      if(scanner.hasNextLine()) {
        println("we got input")
        input = scanner.nextLine().lowercase().trim()
        println(input)
        if(input == "s") {
          return true
        }
      }
    }
    return false
  }

  private fun runWithCancel(command: Command, scanner: java.util.Scanner): Boolean {
    command.schedule()
    while(command.isScheduled) {
      if(scanner.hasNextLine() && scanner.nextLine().lowercase().trim() == "c") {
        command.cancel()
        return false
      }
    }
    return true
  }

  fun runAutoTests(): Command {
    val premoveTests = SequentialCommandGroup()
    generatePremoveTests().forEach { premoveTests.addCommands(it) }
    val romTests = SequentialCommandGroup()
    generateROMTests().forEach { romTests.addCommands(it) }

    autotestCommand = runOnce({
      val scanner = java.util.Scanner(System.`in`)
      println("Welcome to autotest. Enter 's' to start testing. Enter anything else to cancel.")
      if (getUserConfirmation(scanner, AutoTestConstants.INPUT_TIMEOUT)) {
        println("Starting range of motion tests, enter c to cancel")
        romTests.schedule()
        if (runWithCancel(romTests, scanner)) {
          println("Enter 's' to start testing specific locations. Enter anything else to cancel.")
          getUserConfirmation(scanner, AutoTestConstants.INPUT_TIMEOUT)
          runWithCancel(romTests, scanner)
        }
      }
      println("autotest is done.")
    })
    return autotestCommand
  }

  companion object {
    fun createSuperstructureManager(robot: Robot): SuperstructureManager {
      return SuperstructureManager(
        robot.elevator,
        robot.pivot,
        robot.wrist,
        robot.drive
      )
    }
  }
}
