package frc.team449.subsystems.wrist

import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.subsystems.superstructure.SuperstructureGoal
import frc.team449.system.motor.createKraken
import java.util.function.Supplier
import kotlin.math.abs

// TODO(the entire class bru)
class Wrist(
  private val motor: TalonFX
) : SubsystemBase() {

  val positionSupplier = Supplier { motor.position.valueAsDouble }
  val velocitySupplier = Supplier { motor.velocity.valueAsDouble }
  val targetSupplier = Supplier { request.Position }

  private val request = MotionMagicVoltage(
    SuperstructureGoal.STOW.wrist.`in`(Radians)
  )

  fun setPosition(position: Double): Command {
    return this.runOnce {
      motor.setControl(
        request
          .withPosition(position)
      )
    } // .until(::atSetpoint)
  }

  fun manualDown(): Command {
    return runOnce { motor.setVoltage(-3.0) }
  }

  fun manualUp(): Command {
    return runOnce { motor.setVoltage(3.0) }
  }

  fun stop(): Command {
    return this.runOnce { motor.stopMotor() }
  }

  private fun atSetpoint(): Boolean {
    return (abs(positionSupplier.get() - request.Position) < WristConstants.TOLERANCE)
  }

  override fun periodic() {}

  override fun simulationPeriodic() {
    val motorSimState = motor.simState

    motorSimState.setRawRotorPosition(request.Position / (WristConstants.GEARING * WristConstants.UPR))
  }

  companion object {
    fun createWrist(): Wrist {
      val motor: TalonFX = createKraken(
        id = WristConstants.MOTOR_ID,
        inverted = false,
        sensorToMech = 1 / (WristConstants.GEARING * WristConstants.UPR),
        kP = WristConstants.KP,
        kI = WristConstants.KI,
        kD = WristConstants.KD,
        cruiseVel = WristConstants.CRUISE_VEL,
        maxAccel = WristConstants.MAX_ACCEL,
      )

      return Wrist(motor)
    }
  }
}
