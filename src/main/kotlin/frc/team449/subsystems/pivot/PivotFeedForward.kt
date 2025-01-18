package frc.team449.subsystems.pivot

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import frc.team449.subsystems.elevator.Elevator
import frc.team449.subsystems.elevator.ElevatorConstants
import kotlin.math.cos
import kotlin.math.sign

// Custom Pivot Feedforward based on Elevator Length.
// TODO(Pivot FeedForward)
class PivotFeedForward(
  private val elevator: Elevator,
  private val ks: Double,
  private val kv: Double,
  private val ka: Double,
  kgRetracted: Double,
  kgExtended: Double
) {
  private val map = InterpolatingDoubleTreeMap()

  init {
    map.put(ElevatorConstants.MIN_HEIGHT, kgRetracted)
    map.put(ElevatorConstants.MAX_HEIGHT, kgExtended)
  }

  private fun calculateKg(): Double {
    return map.get(elevator.positionSupplier.get())
  }

  fun calculateWithLength(positionRadians: Double, velocityRadPerSec: Double): Double {
    return this.ks * sign(velocityRadPerSec) + calculateKg() * cos(positionRadians) + this.kv * velocityRadPerSec
  }

  companion object {
    fun createPivotFeedForward(elevator: Elevator): PivotFeedForward {
      return PivotFeedForward(
        elevator,
        PivotConstants.KS,
        PivotConstants.KV,
        PivotConstants.KA,
        PivotConstants.KG_MIN,
        PivotConstants.KG_MAX,
      )
    }
  }
}
