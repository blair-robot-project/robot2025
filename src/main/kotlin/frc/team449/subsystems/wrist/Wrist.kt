package frc.team449.subsystems.wrist

import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.subsystems.SuperstructureConstants
import frc.team449.system.motor.createKraken
import java.util.function.Supplier

// TODO(the entire class bru)
class Wrist(
  private val motor: TalonFX
) : SubsystemBase() {

  val positionSupplier = Supplier { motor.position.valueAsDouble }
  val velocitySupplier = Supplier { motor.velocity.valueAsDouble }
  val targetSupplier = Supplier { request.Position }

  val request = MotionMagicVoltage(SuperstructureConstants.STOW_POSITIONS.third)


  fun setPosition(position: Double): Command {
    return this.runOnce {
      motor.setControl(
        request
          .withPosition(position)
      )
    }
  }

  companion object {
    fun createWrist(): Wrist {
      val motor: TalonFX = createKraken(WristConstants.MOTOR_ID, false)

      return Wrist(motor)
    }
  }
}
