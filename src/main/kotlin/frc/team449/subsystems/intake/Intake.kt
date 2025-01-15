package frc.team449.subsystems.intake

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.createKraken

// TODO(the entire class bru)
class Intake(
  private val motor1: TalonFX,
  private val motor2: TalonFX,
  private val motor3: TalonFX,
  private val motor4: TalonFX
) : SubsystemBase() {

  fun intakeCoral(): Command {
    return PrintCommand("Intaking Coral")
  }

  fun intakeAlgae(): Command {
    return PrintCommand("Intaking Algae")
  }

  companion object {
    fun createIntake(): Intake {
      val motor1 = createKraken(IntakeConstants.MOTOR_ONE_ID, IntakeConstants.MOTOR_ONE_INVERTED)
      val motor2 = createKraken(IntakeConstants.MOTOR_TWO_ID, IntakeConstants.MOTOR_TWO_INVERTED)
      val motor3 = createKraken(IntakeConstants.MOTOR_THREE_ID, IntakeConstants.MOTOR_THREE_INVERTED)
      val motor4 = createKraken(IntakeConstants.MOTOR_FOUR_ID, IntakeConstants.MOTOR_FOUR_INVERTED)

      return Intake(motor1, motor2, motor3, motor4)
    }
  }
}
