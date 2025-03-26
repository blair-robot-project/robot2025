package frc.team449.subsystems.superstructure

import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.FunctionalCommand
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
  var userInput = false
  var runningTest = false

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

  //BITs
  private fun checkVoltageWait(boolSupplier: BooleanSupplier, dblSupplier: DoubleSupplier, highVoltageVal: Double): Command {
    return FunctionalCommand(
      {}, {
        if(dblSupplier.asDouble > highVoltageVal) {
          println("Voltage is High! Currently Reading: " + dblSupplier.asDouble)
        } }, {},
      boolSupplier
    )
  }

  private fun runTest(name: String, setpoint: Double, slowDeadline: Double, realDeadline: Double, setpointName: String): Command {
    return Commands.sequence(
      InstantCommand({
        timer.reset()
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

  private fun generatePositionTests(): List<Command> {
    val pivotTests = SequentialCommandGroup()
    listOf(
      BITConstants.PIVOT_SETPOINT_ONE,
      BITConstants.PIVOT_SETPOINT_TWO,
      BITConstants.PIVOT_SETPOINT_THREE,
      BITConstants.PIVOT_SETPOINT_FOUR,
      BITConstants.PIVOT_SETPOINT_FIVE,
      BITConstants.PIVOT_SETPOINT_SIX
    ).forEach { pivotTests.addCommands(runTest("pivot", it, BITConstants.PIVOT_EXPECTED_TIME,
      BITConstants.PIVOT_TIMEOUT, "the angle ${Units.radiansToDegrees(it)} in degrees")) }

    val elevatorTests = SequentialCommandGroup()
    listOf(
      BITConstants.ELEVATOR_SETPOINT_ONE,
      BITConstants.ELEVATOR_SETPOINT_TWO,
      BITConstants.ELEVATOR_SETPOINT_THREE,
      BITConstants.ELEVATOR_SETPOINT_FOUR
    ).forEach { pivotTests.addCommands(runTest("elevator", it, BITConstants.ELEVATOR_EXPECTED_TIME,
      BITConstants.ELEVATOR_TIMEOUT, "the height $it in meters")) }

    val wristTests = SequentialCommandGroup()
    listOf(
      BITConstants.WRIST_SETPOINT_ONE,
      BITConstants.WRIST_SETPOINT_TWO,
      BITConstants.WRIST_SETPOINT_THREE,
      BITConstants.WRIST_SETPOINT_FOUR,
      BITConstants.WRIST_SETPOINT_FIVE
    ).forEach { pivotTests.addCommands(runTest("wrist", it, BITConstants.WRIST_EXPECTED_TIME,
      BITConstants.WRIST_TIMEOUT, "the angle ${Units.radiansToDegrees(it)} in degrees")) }

    return listOf(
      pivotTests,
      WaitCommand(BITConstants.EXTERNAL_WAIT),
      elevatorTests,
      WaitCommand(BITConstants.EXTERNAL_WAIT),
      wristTests,
      WaitCommand(BITConstants.EXTERNAL_WAIT),
      requestGoal(SuperstructureGoal.STOW)
      )
  }

  private fun doPivotROM(cruiseVel: AngularVelocity, maxAccel: AngularAcceleration): Command {
    return Commands.sequence(
      InstantCommand({
        PivotConstants.changeMaxAccel(maxAccel)
        PivotConstants.changeCruiseVel(cruiseVel)
      }),
      pivot.setPosition(BITConstants.PIVOT_HARDSTOP_FRONT - Units.degreesToRadians(1.0)),
      checkVoltageWait({ pivot.atSetpoint(BITConstants.PIVOT_TOLERANCE) }, {pivot.getMotorVoltage()}, BITConstants.HIGH_PIVOT_VOLTAGE),
      pivot.setPosition(BITConstants.PIVOT_HARDSTOP_BACK + Units.degreesToRadians(1.0)),
      checkVoltageWait({ pivot.atSetpoint(BITConstants.PIVOT_TOLERANCE) }, {pivot.getMotorVoltage()}, BITConstants.HIGH_PIVOT_VOLTAGE)
    )
  }

  private fun doElevatorROM(cruiseVel: Double, maxAccel: Double): Command {
    return Commands.sequence(
      InstantCommand({
        ElevatorConstants.changeMaxAccel(maxAccel)
        ElevatorConstants.changeCruiseVel(cruiseVel)
      }),
      elevator.setPosition(SuperstructureGoal.L4.elevator.`in`(Meters) + Units.inchesToMeters(2.0)),
      checkVoltageWait({ elevator.atSetpoint(BITConstants.ELEVATOR_TOLERANCE) }, {elevator.getMotorVoltage()}, BITConstants.HIGH_ELEVATOR_VOLTAGE),
      elevator.setPosition(SuperstructureGoal.L1.elevator.`in`(Meters)),
      checkVoltageWait({ elevator.atSetpoint(BITConstants.ELEVATOR_TOLERANCE) }, {elevator.getMotorVoltage()}, BITConstants.HIGH_ELEVATOR_VOLTAGE),
    )
  }

  private fun doWristROM(cruiseVel: AngularVelocity, maxAccel: AngularAcceleration): Command {
    return Commands.sequence(
      InstantCommand({
        WristConstants.changeMaxAccel(maxAccel)
        WristConstants.changeCruiseVel(cruiseVel)
      }),
      wrist.setPosition(BITConstants.WRIST_HARDSTOP_FRONT - Units.degreesToRadians(1.0)),
      checkVoltageWait({ wrist.atSetpoint(BITConstants.WRIST_TOLERANCE) }, {wrist.getMotorVoltage()}, BITConstants.HIGH_WRIST_VOLTAGE),
      wrist.setPosition(BITConstants.WRIST_HARDSTOP_BACK + Units.degreesToRadians(1.0)),
      checkVoltageWait({ wrist.atSetpoint(BITConstants.WRIST_TOLERANCE) }, {wrist.getMotorVoltage()}, BITConstants.HIGH_WRIST_VOLTAGE)
    )
  }

  private fun generateROMTests(): List<Command> {
    val pivotRomTest = Commands.sequence(
      doPivotROM(PivotConstants.CRUISE_VEL_VALUE / BITConstants.PIVOT_FIRST_ROM_DIVIDE, PivotConstants.MAX_ACCEL_VALUE / BITConstants.PIVOT_FIRST_ROM_DIVIDE),
      doPivotROM(PivotConstants.CRUISE_VEL_VALUE / BITConstants.PIVOT_SECOND_ROM_DIVIDE, PivotConstants.MAX_ACCEL_VALUE / BITConstants.PIVOT_SECOND_ROM_DIVIDE),
      doPivotROM(PivotConstants.CRUISE_VEL_VALUE, PivotConstants.MAX_ACCEL_VALUE),
      pivot.setPosition(BITConstants.PIVOT_SETPOINT_SIX),
      WaitUntilCommand { pivot.atSetpoint() },
    )

    val elevatorRomTest = Commands.sequence(
      doElevatorROM(ElevatorConstants.CRUISE_VEL_VALUE / BITConstants.ELEVATOR_FIRST_ROM_DIVIDE, ElevatorConstants.MAX_ACCEL_VALUE / BITConstants.ELEVATOR_FIRST_ROM_DIVIDE),
      doElevatorROM(ElevatorConstants.CRUISE_VEL_VALUE / BITConstants.ELEVATOR_SECOND_ROM_DIVIDE, ElevatorConstants.MAX_ACCEL_VALUE / BITConstants.ELEVATOR_SECOND_ROM_DIVIDE),
      doElevatorROM(ElevatorConstants.CRUISE_VEL_VALUE, ElevatorConstants.MAX_ACCEL_VALUE),
      requestGoal(SuperstructureGoal.STOW)
    )

    val wristRomTest = Commands.sequence(
      doWristROM(WristConstants.CRUISE_VEL_VALUE / BITConstants.WRIST_FIRST_ROM_DIVIDE, WristConstants.MAX_ACCEL_VALUE / BITConstants.WRIST_FIRST_ROM_DIVIDE),
      doWristROM(WristConstants.CRUISE_VEL_VALUE / BITConstants.WRIST_SECOND_ROM_DIVIDE, WristConstants.MAX_ACCEL_VALUE / BITConstants.WRIST_SECOND_ROM_DIVIDE),
      doWristROM(WristConstants.CRUISE_VEL_VALUE, WristConstants.MAX_ACCEL_VALUE),
      requestGoal(SuperstructureGoal.STOW)
    )
      
    return listOf(
      pivotRomTest,
      WaitCommand(BITConstants.EXTERNAL_WAIT),
      elevatorRomTest,
      WaitCommand(BITConstants.EXTERNAL_WAIT),
      wristRomTest,
      WaitCommand(BITConstants.EXTERNAL_WAIT),
      requestGoal(SuperstructureGoal.STOW),
      WaitCommand(BITConstants.EXTERNAL_WAIT)
      )
    
  }

  fun runBITs(): Command {
    val positionTests = SequentialCommandGroup()
    generatePositionTests().forEach { positionTests.addCommands(it) }
    val romTests = SequentialCommandGroup()
    generateROMTests().forEach { romTests.addCommands(it) }

    return Commands.sequence(
      InstantCommand({
        runningTest = true
        userInput = false
        println("Welcome to BITs. Press d-pad down to start.")
        timer.reset()
      }),
      WaitUntilCommand { userInput || timer.get() > BITConstants.INPUT_TIMEOUT },
      ConditionalCommand(
        Commands.sequence(
          InstantCommand({ userInput = false }),
          PrintCommand("Starting ROM tests. Press d-pad at any time to cancel."),
          romTests.onlyWhile { !userInput },
          ConditionalCommand(

            Commands.sequence(
              PrintCommand("ROM tests finished. Press d-pad to start selective location testing."),
              InstantCommand({ timer.reset() }),
              WaitUntilCommand { userInput || timer.get() > BITConstants.INPUT_TIMEOUT },
              ConditionalCommand(

                Commands.sequence(
                  PrintCommand("Starting selective location testing. Press d-pad at any time to cancel."),
                  InstantCommand({userInput = false}),
                  positionTests.onlyWhile { !userInput },
                  ConditionalCommand(

                    Commands.sequence(
                      PrintCommand("Selective location testing finished. Press d-pad to start reef level testing."),
                      InstantCommand({ timer.reset() }),
                      WaitUntilCommand { userInput || timer.get() > BITConstants.INPUT_TIMEOUT },
                      ConditionalCommand(

                        Commands.sequence(
                          InstantCommand({ userInput = false }),
                          Commands.sequence(
                            requestGoal(SuperstructureGoal.L2),
                            requestGoal(SuperstructureGoal.L4),
                            requestGoal(SuperstructureGoal.L1),
                            requestGoal(SuperstructureGoal.L3)
                          ).onlyWhile({!userInput})

                        ),
                        PrintCommand("BITs canceled from lack of input."),
                        { userInput }
                      )
                    ),
                    PrintCommand("BITs canceled.")
                  ) { !userInput }),
                PrintCommand("BITs canceled from lack of input."),
              ) { userInput }
            ),
            PrintCommand("BITs canceled.")
          ) { !userInput },
        ),
        PrintCommand("BITs canceled from lack of input..")
      ) { userInput }
    ).finallyDo(Runnable {
      runningTest = false
      userInput = false
    })

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
