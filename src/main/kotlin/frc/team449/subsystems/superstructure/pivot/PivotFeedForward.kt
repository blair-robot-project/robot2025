package frc.team449.subsystems.superstructure.pivot

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import frc.team449.subsystems.superstructure.elevator.Elevator
import kotlin.math.cos

// Custom Pivot Feedforward based on Elevator Length.
// TODO(Pivot FeedForward)
class PivotFeedForward(
  private val elevator: Elevator,
  kgRetracted: Double,
  kgExtended: Double
) {
  private val map = InterpolatingDoubleTreeMap()

  init {
    map.put(0.0, kgRetracted)
    map.put(PivotConstants.KG_MAX_EXTENSION, kgExtended)
  }

  private fun calculateKg(): Double {
    return map.get(elevator.positionSupplier.get())
  }

  fun calculateWithLength(positionRadians: Double): Double {
    return calculateKg() * cos(positionRadians)
  }

  companion object {
    fun createPivotFeedForward(elevator: Elevator): PivotFeedForward {
      return PivotFeedForward(
        elevator,
        PivotConstants.KG_MIN,
        PivotConstants.KG_MAX,
      )
    }
  }
}
