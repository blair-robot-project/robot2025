package frc.team449.subsystems.superstructure.intake

import com.revrobotics.spark.SparkMax
import dev.doglog.DogLog
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.createSparkMax

class Intake(
  private val motor: SparkMax,
  private val coralInfrared: DigitalInput,
//  private val algaeInfrared: DigitalInput,
//  private val pivotInfrared: DigitalInput,
  private val timer: Timer = Timer()
) : SubsystemBase() {

  private val controller = PIDController(2.5804, 0.0, 0.010)

  private fun setVoltage(voltage: Double): Command {
    return runOnce {
      motor.setVoltage(voltage)
    }
  }

  fun intakeCoral(): Command {
    return setVoltage(IntakeConstants.CORAL_INTAKE_VOLTAGE)
  }

  fun holdCoral(): Command {
    return runOnce {
      controller.reset()
      controller.setpoint = motor.encoder.position
    }
      .andThen(run { motor.setVoltage(controller.calculate(motor.encoder.position)) })
  }

//  fun intakeAlgae(): Command {
//    return setVoltage(IntakeConstants.ALGAE_INTAKE_VOLTAGE)
//  }

//  fun holdAlgae(): Command {
//    return setVoltage(IntakeConstants.ALGAE_HOLD_VOLTAGE)
//  }

  fun descoreAlgae(): Command {
    return setVoltage(IntakeConstants.DESCORE_ALGAE_VOLTAGE)
  }

  fun outtakeL1(): Command {
    return setVoltage(IntakeConstants.L1_OUTTAKE)
  }

  fun outtakeCoral(): Command {
    return setVoltage(IntakeConstants.CORAL_OUTTAKE_VOLTAGE)
  }

  fun outtakeCoralPivot(): Command {
    return setVoltage(IntakeConstants.CORAL_OUTTAKE_PIVOT_SIDE_VOLTAGE)
  }

//  fun outtakeAlgae(): Command {
//    return setVoltage(IntakeConstants.ALGAE_OUTTAKE_VOLTAGE)
//  }

//  fun readyPivotScore(): Command {
//    return runOnce {
//      timer.restart()
//      setVoltage(IntakeConstants.CORAL_HOLD_VOLTAGE).until {
//        if (IntakeConstants.HAS_PIVOT_SIDE_IR_SENSOR) {
//          pivotInfrared.get()
//        } else {
//          timer.get() >= IntakeConstants.READY_PIVOT_CORAL_TIME
//        }
//      }.schedule()
//    }
//  }

  fun coralDetected(): Boolean {
    return !coralInfrared.get()
  }

  fun coralNotDetected(): Boolean {
    return coralInfrared.get()
  }
//
//  fun algaeDetected(): Boolean {
//    return !algaeInfrared.get()
//  }

//  fun hold(): Command {
//    return if (coralDetected()) holdCoral() else holdAlgae()
//  }
//
//  fun pivotCoralDetected(): Boolean {
//    return if (IntakeConstants.HAS_PIVOT_SIDE_IR_SENSOR) !pivotInfrared.get() else false
//  }

  fun stop(): Command {
    return runOnce {
      motor.stopMotor()
    }
  }

//  fun autoIntakeAlgae(): Command {
//    return intakeAlgae().until { algaeDetected() }
//      .andThen(WaitCommand(IntakeConstants.WAIT_AFTER_ALGAE_DETECTED)).andThen(stop())
//  }

  override fun periodic() {
    logData()
  }

  private fun logData() {
    DogLog.log("Intake/Motor Voltage", motor.appliedOutput * 12.0)
    DogLog.log("Intake/Motor Position", motor.encoder.position)
    DogLog.log("Intake/Coral IR sensor", !coralInfrared.get())
//    DogLog.log("Intake/Algae IR sensor", !algaeInfrared.get())
  }

  companion object {
    fun createIntake(): Intake {
      val motor = createSparkMax(
        id = IntakeConstants.MOTOR_ID,
        inverted = IntakeConstants.MOTOR_INVERTED,
        brakeMode = IntakeConstants.BRAKE_MODE,
        currentLimit = IntakeConstants.CURRENT_LIMIT
      )

      val coralSensor = DigitalInput(IntakeConstants.CORAL_SENSOR_DIO_PORT)
//      val algaeSensor = DigitalInput(IntakeConstants.ALGAE_SENSOR_DIO_PORT)
//      val pivotSensor = DigitalInput(IntakeConstants.PIVOT_SENSOR_DIO_PORT)
      return Intake(motor, coralSensor) // , algaeSensor, pivotSensor)
    }
  }
}
