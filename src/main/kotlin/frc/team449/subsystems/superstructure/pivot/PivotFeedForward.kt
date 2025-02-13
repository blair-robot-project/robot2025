package frc.team449.subsystems.superstructure.pivot

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import frc.team449.subsystems.superstructure.elevator.Elevator
import frc.team449.subsystems.superstructure.elevator.ElevatorConstants
import kotlin.math.cos
import kotlin.math.sign

// Custom Pivot Feedforward based on Elevator Length.
// TODO(Pivot FeedForward)
class PivotFeedForward(
  private val elevator: Elevator,
  private val ks: Double,
  private val kv: Double,
  kgRetracted: Double,
  kgExtended: Double
) {
  private val map = InterpolatingDoubleTreeMap()

  init {
    map.put(ElevatorConstants.SIM_MIN_HEIGHT, kgRetracted)
    map.put(ElevatorConstants.SIM_MAX_HEIGHT, kgExtended)
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
        PivotConstants.KG_MIN,
        PivotConstants.KG_MAX,
      )
    }
  }
}
