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
  CENTERED,
  PIVOT,
  OPP
}

class Intake(
  val topMotor: TalonFX, // kraken x60
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
  private var coralPositioned = true
  private var executing = false


  private fun setVoltage(vararg motors: TalonFX, voltage: Double): Command {
    return runOnce {
      motors.forEach { it.setVoltage(voltage) }
    }
  }

  private var allSensorsConfigured = true
  private var lasercanConfigured = listOf<Boolean>()

  init {
    for (sensor in sensors) {
      try {
        sensor.setTimingBudget(LaserCanInterface.TimingBudget.TIMING_BUDGET_20MS)
        sensor.setRegionOfInterest(LaserCanInterface.RegionOfInterest(8, 8, 4, 4))
        sensor.setRangingMode(LaserCanInterface.RangingMode.SHORT)
        lasercanConfigured.plus(true)
      } catch (_: Exception) {
        lasercanConfigured.plus(false)
        allSensorsConfigured = false
      }
    }
  }

  private fun setVoltageTop(voltage: Double): Command {
    return runOnce { topMotor.setVoltage(voltage) }
  }


  private fun setMotorsRight(rightVoltage: Double = IntakeConstants.SIDES_RUN_TO_SIDE_LEFT_VOLTAGE, leftVoltage: Double = rightVoltage) {
    rightMotor.setVoltage(rightVoltage)
    leftMotor.setVoltage(-leftVoltage)
  }

  private fun setMotorsLeft(rightVoltage: Double = IntakeConstants.SIDES_RUN_TO_SIDE_LEFT_VOLTAGE, leftVoltage: Double = rightVoltage) {
    rightMotor.setVoltage(-rightVoltage)
    leftMotor.setVoltage(leftVoltage)
  }

  private fun setMotorsInwards(slowdownConstant: Double = 1.0) {
    topMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE / slowdownConstant)
    rightMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE / slowdownConstant)
    leftMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE / slowdownConstant)
  }

  private fun runIntakeBackwards(slowdownConstant: Double = 1.0) : Command {
    return runOnce {setMotorsInwards(slowdownConstant)}
  }

  private fun runIntakeForwards(slowdownConstant: Double = 1.0) : Command {
    return runOnce {setMotorsOutwards(slowdownConstant)}
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

  private fun holdCoral(): Command {
    return stopMotors().andThen(
      runOnce {
        resetPos()
        topMotor.setControl(PositionVoltage(0.0).withSlot(0))
        rightMotor.setControl(PositionVoltage(0.0).withSlot(0))
        leftMotor.setControl(PositionVoltage(0.0).withSlot(0))
      }
    )
  }

  private fun moveCoralByAmount(distance: Double): Command {
    return runOnce {
      rightMotor.setControl(PositionVoltage(rightMotor.position.valueAsDouble - distance).withSlot(0))
      leftMotor.setControl(PositionVoltage(leftMotor.position.valueAsDouble - distance).withSlot(0))
      topMotor.setControl(PositionVoltage(topMotor.position.valueAsDouble - distance).withSlot(0))
    }.andThen(
      WaitUntilCommand {
        abs(rightMotor.closedLoopError.valueAsDouble) < IntakeConstants.WHEEL_TOLERANCE &&
          abs(leftMotor.closedLoopError.valueAsDouble) < IntakeConstants.WHEEL_TOLERANCE
      }
    ).andThen(holdCoral())
  }

  private fun moveCoralForwardsByAmount(distance: Double): Command {
    return moveCoralByAmount(distance)
  }

  private fun moveCoralBackwardsByAmount(distance: Double): Command {
    return moveCoralByAmount(-distance)
  }

  private fun centerCoral(): Command {
    return runOnce { executing = true }.andThen(
    ConditionalCommand (

      Commands.sequence( // middle sensor on
        runIntakeBackwards(5.0),
        WaitUntilCommand { !middleSensorDetected() },
        moveCoralForwardsByAmount(5.0)
      ),

      Commands.sequence( // middle sensor not on
        runIntakeForwards(5.0),
        WaitUntilCommand { !backSensorDetected() },
        moveCoralBackwardsByAmount(5.0)
      )

    ) { middleSensorDetected() })
      .andThen(runOnce {
        coralPositioned = true
        executing = false
      }).finallyDo( Runnable { executing = false })
  }

  private fun pivotCoralSequence(): Command {
    return Commands.sequence(
      runOnce { executing = true },
      runIntakeBackwards(2.0),
      WaitUntilCommand { !middleSensorDetected() },
      moveCoralForwardsByAmount(3.0),
      WaitUntilCommand { middleSensorDetected() }
    ).andThen(runOnce {
      coralPositioned = true
      executing = false
    }).finallyDo( Runnable { executing = false })
  }

  private fun oppCoralSequence(): Command {
    return Commands.sequence(
      runOnce { executing = true },
      runIntakeForwards(2.0),
      WaitUntilCommand { !backSensorDetected() },
      moveCoralBackwardsByAmount(3.0),
      WaitUntilCommand { backSensorDetected() }
    ).andThen(runOnce {
      coralPositioned = true
      executing = false
    }).finallyDo( Runnable { executing = false })
  }

  private var unverticaling = false
  private var coralIn = true

  private var sensorsOut =
    (
      rightSensor.measurement == null ||
        middleSensor.measurement == null ||
        leftSensor.measurement == null ||
        backSensor.measurement == null
      )

  private var sensorsOutExceptBack =
    (
      (
        rightSensor.measurement == null ||
          middleSensor.measurement == null ||
          leftSensor.measurement == null
        ) &&
        backSensor.measurement != null
      )

  // TODO: check coral motor stall current
  private fun horizontalCoralStall(): Boolean {
    return topMotor.motorStallCurrent.valueAsDouble < 40.0
  }

  private fun verticalCoralStall(): Boolean {
    return if (
      leftMotor.motorStallCurrent.valueAsDouble < 40.0 ||
      rightMotor.motorStallCurrent.valueAsDouble < 40.0
    ) {
      true
    } else {
      false
    }
  }

  fun intakeToHorizontal(): Command {
    return FunctionalCommand(
      {
        unverticaling = false
        coralIn = true // source: trust me bro
        sensorsOut
      },
      {
//        if (sensorsOut) {
//          topMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE)
//        }

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
          if (!unverticaling) {
            topMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE / IntakeConstants.TOP_MOTOR_HORIZONTAL_SLOWDOWN)
            if (middleSensorDetected()) {
              // move left and slow down to prevent overshoot
              setMotorsLeft(IntakeConstants.SIDES_RUN_TO_SIDE_LEFT_VOLTAGE / IntakeConstants.SIDES_SLOWDOWN_CONSTANT * 1.65)
            } else {
              // move left
              setMotorsLeft()
            }
          } else { // unverticaling

            if (onlyRightSensor()) {
              // if it's only the right sensor we've successfully gotten the coral horizontal
              // so end the process
              unverticaling = false
            } else {
              // coral should be diagonal right now in between middle and right sensor
              // move right until just right sensor, pulling in slightly for steadiness
              topMotor.setVoltage(IntakeConstants.TOP_CORAL_OUTTAKE_VOLTAGE / IntakeConstants.TOP_MOTOR_HORIZONTAL_SLOWDOWN)
              setMotorsRight()
            }
          }
        }

        if (leftSensorDetected()) {
          topMotor.setVoltage(IntakeConstants.TOP_CORAL_INWARDS_VOLTAGE / IntakeConstants.TOP_MOTOR_HORIZONTAL_SLOWDOWN)
          if (middleSensorDetected()) {
            // move right and slow down to prevent overshoot
            setMotorsRight(IntakeConstants.SIDES_RUN_TO_SIDE_LEFT_VOLTAGE / IntakeConstants.SIDES_SLOWDOWN_CONSTANT)
          } else {
            // move right
            setMotorsRight()
          }
        }

        if (onlyMiddleSensor()) {
          if (!backSensorDetected()) {
            if (unverticaling) {
              // move right and out slowly
              setMotorsRight()
              topMotor.setVoltage(IntakeConstants.TOP_CORAL_OUTTAKE_VOLTAGE / 3)
            } else {
              // run inwards till we hit back sensor
              setMotorsInwards()
            }
          } else { // hitting back sensor
            unverticaling = true
            // move right and out slowly
            setMotorsRight()
            topMotor.setVoltage(IntakeConstants.TOP_CORAL_OUTTAKE_VOLTAGE / 3)
          }
        }
      },
      { },
      {
//        if (sensorsOut) { horizontalCoralStall() } else { coralIsHorizontal() }
        coralIsHorizontal()
      }
    ).andThen(changePieceToCoral(true)).andThen(runOnce { topMotor.setVoltage(IntakeConstants.TOP_L1_HOLD) })
  }

  fun intakeToVertical(): Command {
    return FunctionalCommand(
      { },
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
        backSensorDetected()
      },
    ).andThen(changePieceToCoral(false)).andThen(holdCoral())
  }

  fun intakeAlgae(): Command {
    return Commands.sequence(
      runOnce { topMotor.configurator.apply(IntakeConstants.TOP_MOTOR_INTAKING_CONFIG) },
      setVoltageTop(IntakeConstants.ALGAE_INTAKE_VOLTAGE),
      changePieceToAlgae(),
    )
  }

   fun holdAlgae(): Command {
    return Commands.sequence(
      runOnce { topMotor.configurator.apply(IntakeConstants.TOP_MOTOR_HOLDING_CONFIG) },
      setVoltageTop(IntakeConstants.ALGAE_HOLD_VOLTAGE),
    )
  }


  fun holdAlgaeProc(): Command {
    return Commands.sequence(
      runOnce { topMotor.configurator.apply(IntakeConstants.TOP_MOTOR_HOLDING_CONFIG_PROC) },
      setVoltageTop(IntakeConstants.ALGAE_HOLD_VOLTAGE),
    )
  }


  fun outtakeL1(): Command {
    return Commands.sequence(
      setVoltageTop(IntakeConstants.TOP_L1_OUTTAKE),
      changePieceToNone(),
    )
  }

  fun outtakeCoral(): Command {
    return Commands.sequence(
      runOnce { setMotorsOutwards(-1.0) },
      changePieceToNone(),
    )
  }

  fun outtakeCoralPivot(): Command {
    return Commands.sequence(
      runOnce { setMotorsOutwards() },
      changePieceToNone()
    )
  }

  fun outtakeAlgae(): Command {
    return Commands.sequence(
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

  private fun onlyRightSensor(): Boolean {
    return !laserCanDetected(leftSensor) && laserCanDetected(rightSensor) && !laserCanDetected(middleSensor)
  }

  private fun onlyLeftSensor(): Boolean {
    return laserCanDetected(leftSensor) && !laserCanDetected(rightSensor) && !laserCanDetected(middleSensor)
  }

  private fun onlyMiddleSensor(): Boolean {
    return !laserCanDetected(leftSensor) && !laserCanDetected(rightSensor) && laserCanDetected(middleSensor)
  }

  fun algaeDetected(): Boolean {
    return gamePiece == Piece.ALGAE
  }

  private var algaeDebouncer = Debouncer(0.2,Debouncer.DebounceType.kRising)
  private var resetAlgaeDebouncer = WaitUntilCommand{algaeDebouncer.calculate(false)}.ignoringDisable(true)

  private fun changePieceToAlgae(): Command {
    return WaitUntilCommand {
      algaeDebouncer.calculate(topMotor.statorCurrent.valueAsDouble >
        IntakeConstants.ALGAE_STALL_VOLTAGE_THRESHOLD)
    }.onlyIf { RobotBase.isReal() }
      .andThen(runOnce { gamePiece = Piece.ALGAE })
      .andThen(holdAlgae())
//     .andThen(runOnce {resetAlgaeDebouncer.schedule()}) // reset
  }

  private fun changePieceToCoral(horizontal: Boolean): Command {
    return runOnce {
      gamePiece = if (horizontal) {
        Piece.CORAL_HORIZONTAL
      } else {
        Piece.CORAL_VERTICAL
      }
      coralPos = CoralPlace.CENTERED
    }
  }

  fun recenterCoral() : Command {
    return runOnce { coralPositioned = false}
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
          }.andThen(WaitCommand(IntakeConstants.WAIT_BEFORE_ALGAE_OUT))

        ) { coralOuttaken }.onlyIf { RobotBase.isReal() },
        WaitCommand(2.0)
      ),
      runOnce { gamePiece = Piece.NONE },
      stopMotors()
    )
  }

  fun moveCoralOppSide(): Command {
    return runOnce {
      coralPos = CoralPlace.OPP
      coralPositioned = false
    }
  }

  fun moveCoralPivotSide(): Command {
    return runOnce {
      coralPos = CoralPlace.PIVOT
      coralPositioned = false
    }
  }

  fun hasCoral(): Boolean {
    return gamePiece == Piece.CORAL_HORIZONTAL || gamePiece == Piece.CORAL_VERTICAL
  }

  fun hasPiece(): Boolean {
    return algaeDetected() || coralDetected()
  }

  fun resetPiece(): Command {
    return runOnce {
      gamePiece = Piece.CORAL_VERTICAL
    }
  }

  fun stopMotors(): Command {
    return runOnce {
      topMotor.stopMotor()
      rightMotor.stopMotor()
      leftMotor.stopMotor()
    }
  }

  override fun periodic() {
    if(!coralDetected()) {
      gamePiece = Piece.NONE
    }
    logData()
  }


  fun monitorCoral() : Command {
    val f =  FunctionalCommand (
      //initialization
      { executing = false },
      //execute
      {
        if(coralIsVertical() && !executing) {
          if(!coralPositioned || !middleSensorDetected() || !backSensorDetected()) {
            when (coralPos) {
              CoralPlace.CENTERED -> {
                centerCoral().schedule()
              }
              CoralPlace.PIVOT -> {
                pivotCoralSequence().schedule()
              }
              else -> {
                oppCoralSequence().schedule()
              }
            }
          }
        } else {
          //do nothing
        }
      },
      //on end
      { },
      //finish requirements
      { false }
    )
    f.addRequirements(this)
    return f
  }

  private fun logData() {
    KrakenDogLog.log("Intake/topMotor", topMotor)
    DogLog.log("Intake/topMotorTemperature", topMotor.deviceTemp.valueAsDouble)
    KrakenDogLog.log("Intake/rightMotor", rightMotor)
    KrakenDogLog.log("Intake/leftMotor", leftMotor)

    DogLog.log("Intake/HorizontalIntake", coralIsHorizontal())
    DogLog.log("Intake/VerticalIntake", coralIsVertical())

    val back: LaserCanInterface.Measurement? = backSensor.measurement
    val left: LaserCanInterface.Measurement? = leftSensor.measurement
    val right: LaserCanInterface.Measurement? = rightSensor.measurement
    val middle: LaserCanInterface.Measurement? = middleSensor.measurement
    DogLog.log("Intake/LaserCan/Distance(mm)/Back Sensor", back?.distance_mm?.toDouble() ?: -1.0)
    DogLog.log("Intake/LaserCan/Distance(mm)/Left Sensor", left?.distance_mm?.toDouble() ?: -1.0)
    DogLog.log("Intake/LaserCan/Distance(mm)/Right Sensor", right?.distance_mm?.toDouble() ?: -1.0)
    DogLog.log("Intake/LaserCan/Distance(mm)/Middle Sensor", middle?.distance_mm?.toDouble() ?: -1.0)

    DogLog.log("Intake/LaserCan/ Detecting/ Back", laserCanDetected(backSensor))
    DogLog.log("Intake/LaserCan/ Detecting/ Right", laserCanDetected(rightSensor))
    DogLog.log("Intake/LaserCan/ Detecting/ Left", laserCanDetected(leftSensor))
    DogLog.log("Intake/LaserCan/ Detecting/ Middle", laserCanDetected(middleSensor))

    DogLog.log("Intake/LaserCan/ Connection/ Middle", laserCanIsPlugged(middleSensor))
    DogLog.log("Intake/LaserCan/ Connection/ Right", laserCanIsPlugged(rightSensor))
    DogLog.log("Intake/LaserCan/ Connection/ Left", laserCanIsPlugged(leftSensor))
    DogLog.log("Intake/LaserCan/ Connection/ Back", laserCanIsPlugged(backSensor))

    DogLog.log("Intake/LaserCan/All Sensors Configured", allSensorsConfigured)
    DogLog.log("Intake/LaserCan/ Configured list", lasercanConfigured.toBooleanArray())

    val pieceName = when (gamePiece) {
      Piece.NONE -> "none"
      Piece.ALGAE -> "algae"
      Piece.CORAL_VERTICAL -> "coral vertical"
      Piece.CORAL_HORIZONTAL -> "coral horizontal"
    }
    DogLog.log("Intake/Piece", pieceName)
    DogLog.log("Intake/Voltage", topMotor.motorVoltage.valueAsDouble)

    DogLog.log("Intake/Recenter/executing", executing)
    DogLog.log("Intake/Recenter/coral positioned", coralPositioned)
    DogLog.log("Intake/Recenter/close loop error", rightMotor.closedLoopError.valueAsDouble)
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
