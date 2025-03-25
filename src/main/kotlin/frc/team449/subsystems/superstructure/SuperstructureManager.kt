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
import edu.wpi.first.wpilibj2.command.PrintCommand
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

  private fun runTest(name: String, setpoint: Double, slowDeadline: Double, realDeadline: Double, setpointName: String): Command {
    return runOnce({
      timer.restart()
      autotestStart = when (name) {
        "pivot" -> "${Units.radiansToDegrees(pivot.positionSupplier.get())} degrees"
        "elevator" -> "${Units.radiansToDegrees(elevator.positionSupplier.get())} meters"
        else -> "${Units.radiansToDegrees(wrist.positionSupplier.get())} degrees"
      }
    }).andThen(Commands.sequence(
      when (name) {
        "pivot" -> pivot.setPosition(setpoint)
        "elevator" -> elevator.setPosition(setpoint)
        else -> wrist.setPosition(setpoint)
      },
      WaitUntilCommand(
        when (name) {
          "pivot" -> BooleanSupplier { pivot.atSetpoint(AutoTestConstants.PIVOT_TOLERANCE) }
          "elevator" -> BooleanSupplier { elevator.atSetpoint(AutoTestConstants.ELEVATOR_TOLERANCE) }
          else -> BooleanSupplier { wrist.atSetpoint(AutoTestConstants.WRIST_TOLERANCE) }
        }
      )
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
        "pivot" -> AutoTestConstants.WAIT_BETWEEN_PIVOT_TESTS
        "elevator" -> AutoTestConstants.WAIT_BETWEEN_ELEVATOR_TESTS
        else -> AutoTestConstants.WAIT_BETWEEN_WRIST_TESTS
      })
    ))
  }

  private fun setUpPremoveTests(): List<SequentialCommandGroup> {
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
      elevatorTests,
      wristTests
    )
  }

  private fun doPivotROM(cruiseVel: AngularVelocity, maxAccel: AngularAcceleration): Command {
    return Commands.sequence(
      InstantCommand({
        PivotConstants.changeMaxAccel(maxAccel)
        PivotConstants.changeCruiseVel(cruiseVel)
      }),
      pivot.setPosition(AutoTestConstants.PIVOT_HARDSTOP_FRONT - Units.degreesToRadians(1.0)),
      WaitUntilCommand { pivot.atSetpoint(AutoTestConstants.PIVOT_TOLERANCE) },
      pivot.setPosition(AutoTestConstants.PIVOT_HARDSTOP_BACK + Units.degreesToRadians(1.0)),
      WaitUntilCommand { pivot.atSetpoint(AutoTestConstants.PIVOT_TOLERANCE) }
    )
  }

  private fun doElevatorROM(cruiseVel: Double, maxAccel: Double): Command {
    return Commands.sequence(
      InstantCommand({
        ElevatorConstants.changeMaxAccel(maxAccel)
        ElevatorConstants.changeCruiseVel(cruiseVel)
      }),
      elevator.setPosition(SuperstructureGoal.L4.elevator.`in`(Meters) + Units.inchesToMeters(2.0)),
      WaitUntilCommand { elevator.atSetpoint(AutoTestConstants.ELEVATOR_TOLERANCE) },
      elevator.setPosition(SuperstructureGoal.L1.elevator.`in`(Meters)),
      WaitUntilCommand { elevator.atSetpoint(AutoTestConstants.ELEVATOR_TOLERANCE) },
    )
  }

  private fun doWristROM(cruiseVel: AngularVelocity, maxAccel: AngularAcceleration): Command {
    return Commands.sequence(
      InstantCommand({
        WristConstants.changeMaxAccel(maxAccel)
        WristConstants.changeCruiseVel(cruiseVel)
      }),
      wrist.setPosition(SuperstructureGoal.L4.elevator.`in`(Meters) + Units.inchesToMeters(2.0)),
      WaitUntilCommand { wrist.atSetpoint(AutoTestConstants.WRIST_TOLERANCE) },
      wrist.setPosition(SuperstructureGoal.L1.elevator.`in`(Meters)),
      WaitUntilCommand { wrist.atSetpoint(AutoTestConstants.WRIST_TOLERANCE) },
    )
  }

  private fun generateROMTests(): List<Command> {
    val pivotRomTest = Commands.sequence(
      pivot.setPosition(AutoTestConstants.PIVOT_HARDSTOP_BACK + Units.degreesToRadians(1.0)),
      WaitUntilCommand { pivot.atSetpoint() },
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
      elevatorRomTest,
      requestGoal(SuperstructureGoal.STOW)
    )
    
  }

  fun runAutoTests(): Command {
    val tests = setUpPremoveTests()
    val romTests = SequentialCommandGroup()
    generateROMTests().forEach { romTests.addCommands(it) }
    return Commands.sequence(
      romTests,
      requestGoal(SuperstructureGoal.STOW),
      tests[0],
      WaitCommand(AutoTestConstants.WAIT_BETWEEN_EXTERNAL_TESTS),
      tests[1],
      WaitCommand(AutoTestConstants.WAIT_BETWEEN_EXTERNAL_TESTS),
      tests[2],
      WaitCommand(AutoTestConstants.WAIT_BETWEEN_EXTERNAL_TESTS),
      requestGoal(SuperstructureGoal.STOW),
      PrintCommand("Autotest is done.")
    )
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
