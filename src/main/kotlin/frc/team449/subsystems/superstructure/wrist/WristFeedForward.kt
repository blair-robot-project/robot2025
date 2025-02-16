package frc.team449.subsystems.superstructure.wrist

import frc.team449.subsystems.superstructure.pivot.Pivot
import kotlin.math.cos
import kotlin.math.sign

// Custom Pivot Feedforward based on Elevator Length.
// TODO(Pivot FeedForward)
class WristFeedForward(
  private val pivot: Pivot,
  private val ks: Double,
  private val kv: Double,
  private val kg: Double
) {

  fun calculate(positionRadians: Double, velocityRadPerSec: Double): Double {
    return this.ks * sign(velocityRadPerSec) + kg * cos(pivot.positionSupplier.get() + positionRadians) + this.kv * velocityRadPerSec
  }

  companion object {
    fun createWristFeedForward(pivot: Pivot): WristFeedForward {
      return WristFeedForward(
        pivot,
        WristConstants.KS,
        WristConstants.KV,
        WristConstants.KG,
      )
    }
  }
}
