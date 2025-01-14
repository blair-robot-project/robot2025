package frc.team449.subsystems.wrist

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

  val request = PositionVoltage(SuperstructureConstants.STOW_POSITIONS.third)

  val feedForward: SimpleMotorFeedforward = SimpleMotorFeedforward(WristConstants.KS, WristConstants.KV, WristConstants.KA)

  private fun setPosition(position: Double): Command {
    return this.runOnce {
      motor.setControl(
        request
          .withPosition(position)
          .withFeedForward(
            feedForward.calculateWithVelocities(velocitySupplier.get(), motor.closedLoopReferenceSlope.valueAsDouble)
          )
      )
    }
  }

  fun stow(): Command {
    return setPosition(SuperstructureConstants.STOW_POSITIONS.third)
  }

  fun L1(): Command {
    return setPosition(SuperstructureConstants.L1_POSITIONS.third)
  }

  fun L2(): Command {
    return setPosition(SuperstructureConstants.L2_POSITIONS.third)
  }

  fun L3(): Command {
    return setPosition(SuperstructureConstants.L3_POSITIONS.third)
  }

  fun L4(): Command {
    return setPosition(SuperstructureConstants.L4_POSITIONS.third)
  }

  companion object {
    fun createWrist(): Wrist {
      val motor: TalonFX = createKraken(WristConstants.MOTOR_ID, false)

      return Wrist(motor)
    }
  }
}
