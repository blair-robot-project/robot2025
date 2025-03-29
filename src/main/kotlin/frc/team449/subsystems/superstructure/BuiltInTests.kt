package frc.team449.subsystems.superstructure
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.*
import frc.team449.Robot
import frc.team449.subsystems.drive.swerve.SwerveModuleKraken
import frc.team449.subsystems.superstructure.elevator.ElevatorConstants
import frc.team449.subsystems.superstructure.pivot.PivotConstants
import frc.team449.subsystems.superstructure.wrist.WristConstants
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
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

  private fun waitUntilDriveAtTolerance(speeds: ChassisSpeeds): Command {
    return InstantCommand({
      robot.drive.set(speeds)
      modulesAtSetpoint = false
    }).andThen(InstantCommand({
      var atSetpoint = true
      robot.drive.modules.forEach() {
        val krakenModule = it as? SwerveModuleKraken //we use krakens
        val angleDistance = abs(krakenModule?.turnController?.setpoint!! - krakenModule.state.angle.radians)
        if(angleDistance >= BITConstants.DRIVE_ANGLE_TOLERANCE) {
          atSetpoint = false
        }
        println(angleDistance)
      }
      modulesAtSetpoint = atSetpoint
    }).until { modulesAtSetpoint })
  }

  fun testDrive(): Command {
    return Commands.sequence(
      InstantCommand({
        val emptyCommand = InstantCommand()
        emptyCommand.addRequirements(drive)
        drive.defaultCommand = emptyCommand
      }),
      waitUntilDriveAtTolerance(ChassisSpeeds(2.0, 0.0, 0.0)),
      waitUntilDriveAtTolerance(ChassisSpeeds(0.0, 2.0, 0.0)),
      waitUntilDriveAtTolerance(ChassisSpeeds(1.0, 1.0, 0.0)),
      waitUntilDriveAtTolerance(ChassisSpeeds(-1.0, -1.0, 0.0)),
      InstantCommand({drive.defaultCommand = robot.driveCommand})
    )
  }

  fun testIntake(): Command {
    return Commands.sequence(
      intake.intakeCoral(),
      WaitCommand(2.0),
      intake.outtakeCoral(),
      WaitCommand(1.0),
      intake.stop()
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
      manager.requestGoal(SuperstructureGoal.STOW)
    )
  }

  private fun runROMTest(
    voltageSet: Runnable,
    negVoltageSet: Runnable,
    atFrontSupplier: BooleanSupplier,
    atBackSupplier: BooleanSupplier,
    stopCommand: Command
  ): Command {
    return Commands.sequence(
      InstantCommand(voltageSet),
      WaitUntilCommand(atFrontSupplier),
      InstantCommand(negVoltageSet),
      WaitUntilCommand(atBackSupplier),
      stopCommand
    )
  }

  private fun generateROMTests(): List<Command> {
    return listOf(
      runROMTest(
        {pivot.setVoltageChar(BITConstants.PIVOT_SLOW_VOLTAGE)},
        {pivot.setVoltageChar(-BITConstants.PIVOT_SLOW_VOLTAGE)},
        {pivot.atSetpoint(BITConstants.PIVOT_HARDSTOP_FRONT)},
        {pivot.atSetpoint(BITConstants.PIVOT_HARDSTOP_BACK)},
        pivot.stop()
      )
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
                      PrintCommand("Selective location testing finished. Press d-pad to start testing scoring positions."),
                      InstantCommand({ timer.reset() }),
                      WaitUntilCommand { userInput || timer.get() > BITConstants.INPUT_TIMEOUT },
                      ConditionalCommand(

                        Commands.sequence(
                          PrintCommand("Testing scoring positions. Press d-pad at any time to cancel."),
                          InstantCommand({ userInput = false }),
                          Commands.sequence(
                            manager.requestGoal(SuperstructureGoal.L2),
                            manager.requestGoal(SuperstructureGoal.L4),
                            manager.requestGoal(SuperstructureGoal.L1),
                            manager.requestGoal(SuperstructureGoal.L3),
                            manager.requestGoal(SuperstructureGoal.STOW)
                          ).onlyWhile { !userInput },
                          PrintCommand("Built in tests are done. Thanks for using!")
                        ),
                        PrintCommand("BITs canceled from lack of input.")
                      ) { userInput }
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
}