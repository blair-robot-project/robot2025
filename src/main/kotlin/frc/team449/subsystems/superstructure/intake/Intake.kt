package frc.team449.subsystems.superstructure.intake

import au.grapplerobotics.LaserCan
import au.grapplerobotics.interfaces.LaserCanInterface
import au.grapplerobotics.simulation.MockLaserCan
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import dev.doglog.DogLog
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotBase.isSimulation
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.Commands
import frc.team449.subsystems.superstructure.intake.IntakeConstants.config
import frc.team449.system.motor.KrakenDogLog
import kotlin.math.abs

enum class Piece {
  CORAL_VERTICAL,
  CORAL_HORIZONTAL,
  ALGAE,
  NONE
}

enum class CoralPlace {
  INTAKEN,
  HORIZONTAL,
  CENTERED,
  PIVOT,
  OPP
}

class Intake(
  private val topMotor: TalonFX, // kraken x60
  private val rightMotor: TalonFX, // kraken x44
  private val leftMotor: TalonFX, // kraken x44
  private val backSensor: LaserCanInterface,
  private val leftSensor: LaserCanInterface,
  private val rightSensor: LaserCanInterface,
  private val middleSensor: LaserCanInterface
) : SubsystemBase() {

  private val sensors = listOf(
    backSensor,
    leftSensor,
    rightSensor,
    middleSensor
  )

  private var gamePiece = Piece.CORAL_VERTICAL
  private var coralPos = CoralPlace.CENTERED
  private var coralPosGoal = CoralPlace.CENTERED

  private var command = "none"
  private var inTolerance = true

  private fun setVoltage(vararg motors: TalonFX, voltage: Double): Command {
    return runOnce {
      motors.forEach { it.setVoltage(voltage) }
    }
  }

  private var allSensorsConfigured = true
  private var lasercanConfigured = listOf<Boolean>()

  init {


      try {
        backSensor.setTimingBudget(LaserCanInterface.TimingBudget.TIMING_BUDGET_20MS)
        for (sensor in sensors) {
          if (sensor != backSensor) {
            sensor.setTimingBudget(LaserCanInterface.TimingBudget.TIMING_BUDGET_50MS)
          }
          sensor.setRegionOfInterest(LaserCanInterface.RegionOfInterest(8, 8, 4, 4))
          sensor.setRangingMode(LaserCanInterface.RangingMode.SHORT)
          lasercanConfigured.plus(true)
        }
      } catch (_: Exception) {
        lasercanConfigured.plus(false)
        allSensorsConfigured = false
      }
    }


  private fun setVoltageTop(voltage: Double): Command {
    return runOnce { topMotor.setVoltage(voltage) }
  }

  private fun setMotorsRight(rightVoltage: Double = IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE, leftVoltage: Double = rightVoltage) {
    rightMotor.setVoltage(rightVoltage)
    leftMotor.setVoltage(-leftVoltage)
  }

  private fun setMotorsLeft(rightVoltage: Double = IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE, leftVoltage: Double = rightVoltage) {
    rightMotor.setVoltage(-rightVoltage)
    leftMotor.setVoltage(leftVoltage)
  }

  private fun setMotorsInwards(slowdownConstant: Double = 1.0) {
    topMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE / slowdownConstant)
    rightMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE / slowdownConstant)
    leftMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE / slowdownConstant)
  }

  private fun runIntakeBackwards(slowdownConstant: Double = 1.0): Command {
    return runOnce { setMotorsInwards(slowdownConstant) }
  }

  private fun runIntakeForwards(slowdownConstant: Double = 1.0): Command {
    return runOnce { setMotorsOutwards(slowdownConstant) }
  }

  fun manualIn(): Command {
    return this.runOnce {
      topMotor.setVoltage(8.0)
      rightMotor.setVoltage(8.0)
      leftMotor.setVoltage(8.0)
    }
  }

  fun manualOut(): Command {
    return this.runOnce {
      topMotor.setVoltage(-5.0)
      rightMotor.setVoltage(-5.0)
      leftMotor.setVoltage(-5.0)
    }
  }

  private fun setMotorsOutwards(slowdownConstant: Double = 1.0) {
    topMotor.setVoltage(IntakeConstants.TOP_CORAL_OUTTAKE_VOLTAGE / slowdownConstant)
    rightMotor.setVoltage(IntakeConstants.TOP_CORAL_OUTTAKE_VOLTAGE / slowdownConstant)
    leftMotor.setVoltage(IntakeConstants.TOP_CORAL_OUTTAKE_VOLTAGE / slowdownConstant)
  }

  fun resetPos() {
    topMotor.setPosition(0.0)
    rightMotor.setPosition(0.0)
    leftMotor.setPosition(0.0)
  }

  private fun positionControlLeft(pos: Double) {
    leftMotor.setControl(PositionVoltage(pos).withSlot(0))
  }

  private fun positionControlRight(pos: Double) {
    rightMotor.setControl(PositionVoltage(pos).withSlot(0))
  }

  private fun positionControlTop(pos: Double) {
    topMotor.setControl(PositionVoltage(pos).withSlot(0))
  }

  private fun holdCoral() {
    resetPos()
    positionControlTop(0.0)
    positionControlRight(0.0)
    positionControlLeft(0.0)
  }

  private fun moveCoralByAmount(distance: Double): Command {
    return Commands.sequence(
      runOnce {
        inTolerance = false
        positionControlTop(rightMotor.position.valueAsDouble - distance)
        positionControlRight(leftMotor.position.valueAsDouble - distance)
        positionControlLeft(topMotor.position.valueAsDouble - distance)
      },
      WaitUntilCommand {
        abs(rightMotor.closedLoopError.valueAsDouble) < IntakeConstants.WHEEL_TOLERANCE &&
          abs(leftMotor.closedLoopError.valueAsDouble) < IntakeConstants.WHEEL_TOLERANCE
      }.withTimeout(0.75),
      runOnce {
        inTolerance = true
        holdCoral()
      }
    )
  }

   fun moveCoralForwardsByAmount(distance: Double): Command {
    return moveCoralByAmount(distance)
  }

   fun moveCoralBackwardsByAmount(distance: Double): Command {
    return moveCoralByAmount(-distance)
  }

  private fun runCoralMovingSequence(intermediateCommand: Command): Command {
    return Commands.sequence(
      intermediateCommand,
      runOnce {
        holdCoral()
        coralPos = coralPosGoal
      }
    )
  }

  private fun centerCoral(): Command {
    return when(coralPos) {
      CoralPlace.INTAKEN -> runCoralMovingSequence(moveCoralForwardsByAmount(IntakeConstants.INTAKEN_TO_CENTERED))
      CoralPlace.OPP -> pivotCoralSequence()
      CoralPlace.PIVOT -> oppCoralSequence()
      else -> InstantCommand()
    }
  }

  private fun pivotCoralSequence(): Command {
    return when(coralPos) {
      CoralPlace.CENTERED -> runCoralMovingSequence(moveCoralBackwardsByAmount(IntakeConstants.PIVOT_MOVEMENT))
      CoralPlace.OPP -> runCoralMovingSequence(moveCoralBackwardsByAmount(IntakeConstants.PIVOT_MOVEMENT + IntakeConstants.OPP_MOVEMENT ))
      else -> InstantCommand()
    }
  }

  private fun oppCoralSequence(): Command {
    return when(coralPos) {
      CoralPlace.CENTERED -> runCoralMovingSequence(moveCoralForwardsByAmount(IntakeConstants.OPP_MOVEMENT))
      CoralPlace.PIVOT -> runCoralMovingSequence(moveCoralForwardsByAmount(IntakeConstants.OPP_MOVEMENT  + IntakeConstants.PIVOT_MOVEMENT))
      else -> InstantCommand()
    }
  }

  private var unverticaling = false
  private var coralIn = true



  fun intakeToHorizontal(): Command {
    return FunctionalCommand(
      {
        command = "intakeToHorizontal"
        coralIn = true // source: trust me bro
        coralPos = CoralPlace.HORIZONTAL
        coralPosGoal = CoralPlace.HORIZONTAL
      },
      {


        if (coralNotDetected()) {
          // pull in coral until a sensor detects
          coralIn = false
          topMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE)
        } else {
          // stop top motor once a coral is detected so we don't have it running when it shouldn't be
          if (!coralIn) {
            coralIn = true
            topMotor.stopMotor()
          }
        }

        if (rightSensorDetected()) {
          topMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE / IntakeConstants.TOP_MOTOR_HORIZONTAL_SLOWDOWN)
          if (middleSensorDetected()) {
            // move left and slow down to prevent overshoot
            setMotorsLeft(IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE / IntakeConstants.SIDES_SLOWDOWN_CONSTANT * 1.65)
          } else {
            // move left
            setMotorsLeft()
          }
        }

        if (leftSensorDetected()) {
          topMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE / IntakeConstants.TOP_MOTOR_HORIZONTAL_SLOWDOWN)
          if (middleSensorDetected()) {
            // move right and slow down to prevent overshoot
            setMotorsRight(IntakeConstants.SIDES_RUN_TO_SIDE_VOLTAGE / IntakeConstants.SIDES_SLOWDOWN_CONSTANT)
          } else {
            // move right
            setMotorsRight()
          }
        }

        if (onlyMiddleSensor()) {
          topMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE / IntakeConstants.TOP_MOTOR_HORIZONTAL_SLOWDOWN)
          setMotorsRight()
        }
      },
      { },
      {
        l1Debouncer.calculate(
          leftSensorDetected() && middleSensorDetected() && rightSensorDetected()
        )
      }
    )
      .andThen(changePieceToL1Coral())
  }

  fun intakeToVertical(): Command {
    return FunctionalCommand(
      {
        command = "intakeToVertical"
        coralPos = CoralPlace.CENTERED
        coralPosGoal = CoralPlace.CENTERED
      },
      {
//        if (sensorsOut) {
//          setMotorsInwards()
//        } else if (sensorsOutExceptBack) {
//          Commands.sequence(
//            inwards().andThen(
//              WaitUntilCommand { backSensorDetected() }
//            )
//          )
//        }

        if (leftSensorDetected() && rightSensorDetected()) {
          // if it's horizontal, just run it right
          // run in a bit because our side motors tweaking lowk
          topMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE / IntakeConstants.TOP_MOTOR_HORIZONTAL_SLOWDOWN)
          setMotorsRight()
        } else {
          // inwards
          setMotorsInwards()
        }
      },
      { },
      {
//        if (sensorsOut) {
//          verticalCoralStall()
//        }else if (sensorsOutExceptBack){
//          backSensorDetected()
//        } else {

        vertDebouncer.calculate(backSensorDetected())
      },
    )
      .andThen(changePieceToVertCoral())
  }

  fun intakeAlgae(): Command {
    return Commands.sequence(
      runOnce {
        command = "intaking algae"
        topMotor.configurator.apply(IntakeConstants.TOP_MOTOR_INTAKING_CONFIG)
      },
      setVoltageTop(IntakeConstants.ALGAE_INTAKE_VOLTAGE),
      changePieceToAlgae(),
    )
  }

  fun holdAlgae(): Command {
    return Commands.sequence(
      runOnce {
        command = "holding algae"
        topMotor.configurator.apply(IntakeConstants.TOP_MOTOR_HOLDING_CONFIG)
      },
      setVoltageTop(IntakeConstants.ALGAE_HOLD_VOLTAGE)
    )
  }

  fun holdAlgaeProc(): Command {
    return Commands.sequence(
      runOnce {
        command = "holding algae proc"
        topMotor.configurator.apply(IntakeConstants.TOP_MOTOR_HOLDING_CONFIG_PROC)
      },
      setVoltageTop(IntakeConstants.ALGAE_HOLD_VOLTAGE),
    )
  }

  fun outtakeL1(): Command {
    return Commands.sequence(
      runOnce { command = "outtaking l1" },
      setVoltageTop(IntakeConstants.TOP_L1_OUTTAKE),
      changePieceToNone(),
    )
  }

  fun outtakeCoral(): Command {
    return Commands.sequence(
      runOnce { command = "outtaking coral" },
      runOnce { setMotorsOutwards(-1.0) },
      changePieceToNone(),
    )
  }

  fun outtakeCoralPivot(): Command {
    return Commands.sequence(
      runOnce { command = "outtaking coral pivot" },
      runOnce { setMotorsOutwards() },
      changePieceToNone()
    )
  }

  fun outtakeAlgae(): Command {
    return Commands.sequence(
      runOnce { command = "outtaking algae" },
      setVoltageTop(IntakeConstants.ALGAE_OUTTAKE_VOLTAGE),
      changePieceToNone(false),
    )
  }

  private fun laserCanDetected(laserCan: LaserCanInterface): Boolean {
    return laserCan.measurement != null && (
      laserCan.measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT &&
        laserCan.measurement.distance_mm <= IntakeConstants.CORAL_DETECTION_THRESHOLD
      )
  }

  private fun laserCanIsPlugged(laserCan: LaserCanInterface): Boolean {
    return laserCan.measurement != null
  }

  fun coralDetected(): Boolean {
    return laserCanDetected(backSensor) || laserCanDetected(leftSensor) || laserCanDetected(rightSensor) || laserCanDetected(middleSensor)
  }

  fun coralNotDetected(): Boolean {
    return !coralDetected()
  }

  private fun coralIsVertical(): Boolean {
    return gamePiece == Piece.CORAL_VERTICAL
  }

  private fun coralIsHorizontal(): Boolean {
    return gamePiece == Piece.CORAL_HORIZONTAL
  }

  private fun rightSensorDetected(): Boolean {
    return laserCanDetected(rightSensor)
  }

  private fun leftSensorDetected(): Boolean {
    return laserCanDetected(leftSensor)
  }

  private fun middleSensorDetected(): Boolean {
    return laserCanDetected(middleSensor)
  }

  private fun backSensorDetected(): Boolean {
    return laserCanDetected(backSensor)
  }

  private fun onlyMiddleSensor(): Boolean {
    return !laserCanDetected(leftSensor) && !laserCanDetected(rightSensor) && laserCanDetected(middleSensor)
  }

  fun hasAlgae(): Boolean {
    return gamePiece == Piece.ALGAE
  }

  private val algaeDebouncer = Debouncer(IntakeConstants.ALGAE_DEBOUNCER_WAIT, Debouncer.DebounceType.kRising)
  private val l1Debouncer = Debouncer(IntakeConstants.L1_DEBOUNCER_WAIT, Debouncer.DebounceType.kRising)
  private val vertDebouncer = Debouncer(IntakeConstants.VERTICAL_DEBOUNCER_WAIT, Debouncer.DebounceType.kRising)
  private val pieceResetDebouncer = Debouncer(IntakeConstants.PIECE_RESET_DEBOUNCER_WAIT, Debouncer.DebounceType.kRising)
  private val pieceDetectDebonucer = Debouncer(IntakeConstants.PIECE_DETECT_DEBOUNCER_WAIT, Debouncer.DebounceType.kRising)

  private fun changePieceToAlgae(): Command {
   return WaitUntilCommand {
//   return   WaitCommand(2.0)
     algaeDebouncer.calculate(
       topMotor.statorCurrent.valueAsDouble >
         IntakeConstants.ALGAE_STALL_VOLTAGE_THRESHOLD
     )
   }.onlyIf { RobotBase.isReal() }
      .andThen(runOnce { gamePiece = Piece.ALGAE })
  }

  private fun changePieceToVertCoral(): Command {
    return runOnce {
      topMotor.stopMotor()
      holdCoral()
      gamePiece = Piece.CORAL_VERTICAL
    }
  }

  private fun changePieceToL1Coral(): Command {
    return runOnce {
      leftMotor.stopMotor()
      rightMotor.stopMotor()
      topMotor.setVoltage(IntakeConstants.TOP_L1_HOLD)
      gamePiece = Piece.CORAL_HORIZONTAL
    }
  }

  private fun changePieceToNone(coralOuttaken: Boolean = true): Command {
    return Commands.sequence(
      Commands.race(
        ConditionalCommand(
          // coral
          WaitUntilCommand { !coralDetected() },

          // algae
          WaitUntilCommand {
            topMotor.statorCurrent.valueAsDouble >
              IntakeConstants.ALGAE_STALL_VOLTAGE_THRESHOLD
          }.andThen(
            WaitCommand(IntakeConstants.WAIT_BEFORE_ALGAE_OUT)
          )

        ) { coralOuttaken }.onlyIf { RobotBase.isReal() },
        // have this wait just so if we never current sense or we're stuck
        // or smth we're not trapped in the outtake command
        WaitCommand(1.0)
      ),
      runOnce { gamePiece = Piece.NONE },
      stopMotorsCmd()
    )
  }

  fun moveCoralOppSide(): Command { return runOnce { coralPosGoal = CoralPlace.OPP } }
  fun moveCoralPivotSide(): Command { return runOnce { coralPosGoal = CoralPlace.PIVOT } }
  fun moveCoralCentered(): Command { return runOnce { coralPosGoal = CoralPlace.CENTERED } }
  //intake to vertical sets coralPosGoal to CoralPlace.CENTERED immediately
  fun moveCoralFromIntake(): Command { return runOnce { coralPos = CoralPlace.INTAKEN } }

  private fun moveTowardsBackSensor(): Command {
    return Commands.sequence(
      runOnce {
        setMotorsInwards(5.0)
      },
      WaitUntilCommand { backSensorDetected() },
      stopMotorsCmd(),
      runOnce { holdCoral() }
    )
  }

  private fun moveTowardsMiddleSensor(): Command {
    return Commands.sequence(
      runOnce {
        setMotorsOutwards(5.0)
      },
      WaitUntilCommand { middleSensorDetected() },
      stopMotorsCmd(),
      runOnce { holdCoral() }
    )
  }

  private fun moveTowardsLeftSensor(): Command {
    return Commands.sequence(
      runOnce {
        setMotorsLeft(2.0)
      },
      stopMotorsCmd(),
      WaitUntilCommand { leftSensorDetected() },
    )
  }

  private fun moveTowardsRightSensor(): Command {
    return Commands.sequence(
      runOnce {
        setMotorsRight(2.0)
      },
      stopMotorsCmd(),
      WaitUntilCommand { rightSensorDetected() },
    )
  }

  fun hasCoral(): Boolean {
    return gamePiece == Piece.CORAL_HORIZONTAL || gamePiece == Piece.CORAL_VERTICAL
  }

  fun hasPiece(): Boolean {
    return hasAlgae() || hasCoral()
  }

  fun resetPiece(): Command {
    return runOnce {
      gamePiece = Piece.CORAL_VERTICAL
    }
  }

  private fun stopMotors() {
    topMotor.stopMotor()
    rightMotor.stopMotor()
    leftMotor.stopMotor()
  }

  fun stopMotorsCmd(): Command {
    return runOnce { stopMotors() }
  }

  override fun periodic() {
    if (hasCoral() && pieceResetDebouncer.calculate(!coralDetected())) {
      gamePiece = Piece.NONE
    }

    if(!hasCoral() && (
      pieceDetectDebonucer.calculate(middleSensorDetected()) ||
      pieceDetectDebonucer.calculate(backSensorDetected())
    )) {
      gamePiece =
        if(leftSensorDetected() || rightSensorDetected()) {
          Piece.CORAL_HORIZONTAL
        } else {
          Piece.CORAL_VERTICAL
        }
    }

    logData()


  }

  fun monitorCoral(): Command {
    val f = FunctionalCommand(
      // initialization
      { command = "monitoring" },
      // execute
      {
        if (coralIsVertical()) {
          if (coralPos != coralPosGoal) {
            //if we're moving to a goal
            when (coralPosGoal) {
              CoralPlace.CENTERED -> {
                command = "centering"
                centerCoral().schedule()
              }
              CoralPlace.PIVOT -> {
                command = "moving to pivot"
                pivotCoralSequence().schedule()
              }
              else -> {
                command = "moving to opp side"
                oppCoralSequence().schedule()
              }
            }

          }

          if (!backSensorDetected()) {
            command = "moving towards back sensor"
            moveTowardsBackSensor().schedule()
          }

          if (!middleSensorDetected()) {
            command = "moving towards middle sensor"
            moveTowardsMiddleSensor().schedule()
          }

        } else if (coralIsHorizontal()) {

          if (!rightSensorDetected()) {
            command = "moving towards right sensor"
            moveTowardsRightSensor().schedule()
          }

          if (!leftSensorDetected()) {
            command = "moving towards left sensor"
            moveTowardsLeftSensor().schedule()
          }

        }
      },
      // on end
      { },
      // finish requirements
      { false }
    )
    f.addRequirements(this)
    return f
  }

  private fun logData() {
    // MOTORS
    KrakenDogLog.log("Intake/Motors/topMotor", topMotor)
    KrakenDogLog.log("Intake/Motors/rightMotor", rightMotor)
    KrakenDogLog.log("Intake/Motors/leftMotor", leftMotor)
    // VOLTAGE
    DogLog.log("Intake/Motors/Voltage/topMotorVoltage", topMotor.motorVoltage.valueAsDouble)
    DogLog.log("Intake/Motors/Voltage/leftMotorVoltage", leftMotor.motorVoltage.valueAsDouble)
    DogLog.log("Intake/Motors/Voltage/rightMotorVoltage", rightMotor.motorVoltage.valueAsDouble)
    // ERROR
    DogLog.log("Intake/Motors/Error/topMotorError", topMotor.closedLoopError.valueAsDouble)
    DogLog.log("Intake/Motors/Error/leftMotorError", leftMotor.closedLoopError.valueAsDouble)
    DogLog.log("Intake/Motors/Error/rightMotorError", rightMotor.closedLoopError.valueAsDouble)
    // TEMP
    DogLog.log("Intake/Motors/Temp/topMotorTemperature", topMotor.deviceTemp.valueAsDouble)
    DogLog.log("Intake/Motors/Temp/leftMotorTemperature", leftMotor.deviceTemp.valueAsDouble)
    DogLog.log("Intake/Motors/Temp/rightMotorTemperature", rightMotor.deviceTemp.valueAsDouble)

    // STATE
    val pieceName = when (gamePiece) {
      Piece.NONE -> "none"
      Piece.ALGAE -> "algae"
      Piece.CORAL_VERTICAL -> "coral vertical"
      Piece.CORAL_HORIZONTAL -> "coral horizontal"
    }
    val place = when (coralPos) {
      CoralPlace.CENTERED -> "centered"
      CoralPlace.PIVOT -> "pivot"
      CoralPlace.OPP -> "opp"
      CoralPlace.HORIZONTAL -> "horizontal"
      CoralPlace.INTAKEN -> "intaken"
    }
    DogLog.log("Intake/State/Piece State", pieceName)
    DogLog.log("Intake/State/Command", command)
    DogLog.log("Intake/State/In Tolerance", inTolerance)
    DogLog.log("Intake/State/Coral Positioned", coralPos == coralPosGoal)
    DogLog.log("Intake/State/Coral Place", place)


    // LASERCANS
    val back: LaserCanInterface.Measurement? = backSensor.measurement
    val left: LaserCanInterface.Measurement? = leftSensor.measurement
    val right: LaserCanInterface.Measurement? = rightSensor.measurement
    val middle: LaserCanInterface.Measurement? = middleSensor.measurement
    DogLog.log("Intake/LaserCan/Distance(mm)/Back Sensor", back?.distance_mm?.toDouble() ?: -1.0)
    DogLog.log("Intake/LaserCan/Distance(mm)/Left Sensor", left?.distance_mm?.toDouble() ?: -1.0)
    DogLog.log("Intake/LaserCan/Distance(mm)/Right Sensor", right?.distance_mm?.toDouble() ?: -1.0)
    DogLog.log("Intake/LaserCan/Distance(mm)/Middle Sensor", middle?.distance_mm?.toDouble() ?: -1.0)

    DogLog.log("Intake/LaserCan/Detecting/Back", laserCanDetected(backSensor))
    DogLog.log("Intake/LaserCan/Detecting/Right", laserCanDetected(rightSensor))
    DogLog.log("Intake/LaserCan/Detecting/Left", laserCanDetected(leftSensor))
    DogLog.log("Intake/LaserCan/Detecting/Middle", laserCanDetected(middleSensor))

    DogLog.log("Intake/LaserCan/Connection/Middle", laserCanIsPlugged(middleSensor))
    DogLog.log("Intake/LaserCan/Connection/Right", laserCanIsPlugged(rightSensor))
    DogLog.log("Intake/LaserCan/Connection/Left", laserCanIsPlugged(leftSensor))
    DogLog.log("Intake/LaserCan/Connection/Back", laserCanIsPlugged(backSensor))

    DogLog.log("Intake/LaserCan/Config/All Sensors Configured", allSensorsConfigured)
    DogLog.log("Intake/LaserCan/Config/Configured List", lasercanConfigured.toBooleanArray())
  }

  companion object {
    fun createIntake(): Intake {
      val topMotor = TalonFX(IntakeConstants.TOP_MOTOR_ID)
      val rightMotor = TalonFX(IntakeConstants.LEFT_MOTOR_ID)
      val leftMotor = TalonFX(IntakeConstants.RIGHT_MOTOR_ID)

      config.Slot0.kP = IntakeConstants.KP
      config.Slot0.kI = IntakeConstants.KI
      config.Slot0.kD = IntakeConstants.KD
      config.Slot0.kS = IntakeConstants.KS

      config.MotorOutput.Inverted = IntakeConstants.TOP_INVERTED
      topMotor.configurator.apply(config)
      topMotor.configurator.apply(IntakeConstants.TOP_MOTOR_INTAKING_CONFIG)
      config.MotorOutput.Inverted = IntakeConstants.LEFT_INVERTED
      leftMotor.configurator.apply(config)
      config.MotorOutput.Inverted = IntakeConstants.RIGHT_INVERTED
      rightMotor.configurator.apply(config)

      // Use MockLaserCan in Simulation
      if (isSimulation()) {
        return Intake(
          topMotor,
          leftMotor,
          rightMotor,
          MockLaserCan(),
          MockLaserCan(),
          MockLaserCan(),
          MockLaserCan()
        )
      }

      topMotor.statorCurrent.setUpdateFrequency(50.0)
      topMotor.position.setUpdateFrequency(50.0)
      topMotor.motorVoltage.setUpdateFrequency(5.0)
      topMotor.closedLoopError.setUpdateFrequency(5.0)
      topMotor.deviceTemp.setUpdateFrequency(5.0)
      topMotor.optimizeBusUtilization()


      leftMotor.statorCurrent.setUpdateFrequency(50.0)
      leftMotor.position.setUpdateFrequency(50.0)
      leftMotor.motorVoltage.setUpdateFrequency(5.0)
      leftMotor.closedLoopError.setUpdateFrequency(5.0)
      leftMotor.deviceTemp.setUpdateFrequency(5.0)
      leftMotor.optimizeBusUtilization()


      rightMotor.statorCurrent.setUpdateFrequency(50.0)
      rightMotor.position.setUpdateFrequency(50.0)
      rightMotor.motorVoltage.setUpdateFrequency(5.0)
      rightMotor.closedLoopError.setUpdateFrequency(5.0)
      rightMotor.deviceTemp.setUpdateFrequency(5.0)
      rightMotor.optimizeBusUtilization()






      return Intake(
        topMotor,
        rightMotor,
        leftMotor,
        LaserCan(IntakeConstants.BACK_CORAL_SENSOR_CAN_ID),
        LaserCan(IntakeConstants.LEFT_CORAL_SENSOR_CAN_ID),
        LaserCan(IntakeConstants.RIGHT_CORAL_SENSOR_CAN_ID),
        LaserCan(IntakeConstants.MIDDLE_CORAL_SENSOR_CAN_ID)
      )
    }
  }
}
