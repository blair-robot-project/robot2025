package frc.team449.subsystems.superstructure.intake

import com.revrobotics.spark.SparkMax
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.createSparkMax

// TODO(the entire class bru)
class Intake(
  private val frontMotor: SparkMax,
  private val backMotor: SparkMax,
  val infrared: DigitalInput
) : SubsystemBase() {

  fun intakeCoral(): Command {
    return runOnce {
      frontMotor.setVoltage(IntakeConstants.CORAL_INTAKE_VOLTAGE)
      backMotor.setVoltage(-IntakeConstants.CORAL_INTAKE_VOLTAGE)
    }
  }

  fun outtakeCoral(): Command {
    return runOnce {
      frontMotor.setVoltage(IntakeConstants.CORAL_OUTTAKE_VOLTAGE)
      backMotor.setVoltage(-IntakeConstants.CORAL_OUTTAKE_VOLTAGE)
    }
  }

  fun intakeAlgae(): Command {
    return runOnce {
      frontMotor.setVoltage(-IntakeConstants.ALGAE_INTAKE_VOLTAGE)
      backMotor.setVoltage(IntakeConstants.ALGAE_INTAKE_VOLTAGE)
    }
  }

  fun outtakeAlgae(): Command {
    return runOnce {
      frontMotor.setVoltage(-IntakeConstants.ALGAE_OUTTAKE_VOLTAGE)
      backMotor.setVoltage(IntakeConstants.ALGAE_OUTTAKE_VOLTAGE)
    }
  }

  fun stop(): Command {
    return runOnce {
      frontMotor.stopMotor()
      backMotor.stopMotor()
    }
  }

  companion object {
    fun createIntake(): Intake {
      val frontMotor = createSparkMax(
        id = IntakeConstants.FRONT_MOTOR_ID,
        inverted = IntakeConstants.FRONT_MOTOR_INVERTED,
        brakeMode = IntakeConstants.BRAKE_MODE,
        currentLimit = IntakeConstants.CURRENT_LIMIT
      )

      val backMotor = createSparkMax(
        id = IntakeConstants.BACK_MOTOR_ID,
        inverted = IntakeConstants.BACK_MOTOR_INVERTED,
        brakeMode = IntakeConstants.BRAKE_MODE,
        currentLimit = IntakeConstants.CURRENT_LIMIT
      )

      val sensor = DigitalInput(IntakeConstants.SENSOR_DIO_PORT)

      return Intake(frontMotor, backMotor, sensor)
    }
  }
}
