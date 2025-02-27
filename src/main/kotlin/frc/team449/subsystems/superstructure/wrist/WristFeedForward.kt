package frc.team449.subsystems.superstructure.wrist

import frc.team449.subsystems.superstructure.pivot.Pivot
import kotlin.math.cos

// Custom Wrist Feedforward based on pivot position.
class WristFeedForward(
  private val pivot: Pivot,
  private val kg: Double
) {

  fun calculate(positionRadians: Double): Double {
    return kg * cos(pivot.positionSupplier.get() + positionRadians)
  }

  companion object {
    fun createWristFeedForward(pivot: Pivot): WristFeedForward {
      return WristFeedForward(
        pivot,
        WristConstants.KG,
      )
    }
  }
}
