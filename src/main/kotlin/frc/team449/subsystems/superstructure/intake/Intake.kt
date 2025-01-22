package frc.team449.subsystems.superstructure.intake

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.createKraken

// TODO(the entire class bru)
class Intake(
  private val frontMotor: TalonFX,
  private val backMotor: TalonFX
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
      val frontMotor = createKraken(
        id = IntakeConstants.FRONT_MOTOR_ID,
        inverted = IntakeConstants.FRONT_MOTOR_INVERTED,
        statorCurrentLimit = IntakeConstants.STATOR_LIMIT,
        burstCurrentLimit = IntakeConstants.SUPPLY_LIMIT,
        burstTimeLimit = Seconds.of(0.0),
        updateFrequency = IntakeConstants.VALUE_UPDATE_FREQ
      )

      val backMotor = createKraken(
        id = IntakeConstants.BACK_MOTOR_ID,
        inverted = IntakeConstants.BACK_MOTOR_INVERTED,
        statorCurrentLimit = IntakeConstants.STATOR_LIMIT,
        burstCurrentLimit = IntakeConstants.SUPPLY_LIMIT,
        burstTimeLimit = Seconds.of(0.0),
        updateFrequency = IntakeConstants.VALUE_UPDATE_FREQ
      )
      return Intake(frontMotor, backMotor)
    }
  }
}
