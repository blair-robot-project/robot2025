package frc.team449.subsystems.elevator

import edu.wpi.first.math.controller.ElevatorFeedforward
import frc.team449.subsystems.pivot.Pivot
import kotlin.math.sign
import kotlin.math.sin

// Custom Elevator Feedforward based on Pivot Angle.
// TODO(Elevator FeedForward!)
class ElevatorFeedForward(
  private val pivot: Pivot,
  private val ks: Double,
  private val kv: Double,
  private val ka: Double,
  private val kg: Double
) : ElevatorFeedforward(ks, kg, kv, ka) {
  override fun calculate(velocity: Double): Double {
    return this.ks * sign(velocity) + this.kg * sin(pivot.positionSupplier.get()) + this.kv * velocity + this.ka * 0.0
  }

  companion object {
    fun createElevatorFeedForward(
      pivot: Pivot
    ): ElevatorFeedForward {
      return ElevatorFeedForward(
        pivot,
        ElevatorConstants.KS,
        ElevatorConstants.KV,
        ElevatorConstants.KA,
        ElevatorConstants.KG
      )
    }
  }
}
