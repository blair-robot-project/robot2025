package frc.team449.subsystems.elevator

import edu.wpi.first.math.controller.ElevatorFeedforward
import frc.team449.subsystems.pivot.Pivot
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sin

// Custom Elevator Feedforward based on Pivot Angle.
// TODO(Elevator FeedForward!)
class ElevatorFeedForward(
  private val pivot: Pivot,
  private val ks: Double,
  private val kv: Double,
  private val kg: Double
) : ElevatorFeedforward(ks, kg, kv) {
  /** FF Model
   *    V = sign(vel) * ks + ((g * sin(angle) - ω^2 * r) / (g)) * kg + vel * kv
   *    Explanation for kg calculation:
   *      kG * sin(angle) is voltage to overcome gravity, but pivot angular velocity adds centripetal acceleration to
   *        the carriage in the direction of the elevator (no matter the pivot angle).
   *      Centripetal acceleration is ω^2 * r, so we find the effective PROPORTION of gravity that this centripetal
   *        acceleration adds, since kG is voltage to overcome 1G.
   *      Thus, this proportion is found by (experienced_gravity - centripetal_accel) / gravity_constant. This makes
   *        our kG apply a voltage to the proportion of gravity that we experience.
   *
   *      This term will effectively become experienced_gravity / gravity_constant or sin(angle) when the pivot velocity
   *        is near 0, which is the standard model of kG * sin(angle)
   *
   *      Note: r in this equation is the distance from the pivot point to the center of mass, not to the carriage. The
   *        center of mass is roughly approximated as carriage_position * 0.75 just based off of vibes
   */
  override fun calculate(velocitySetpoint: Double): Double {
    return this.ks * sign(velocitySetpoint) + this.kg * (
      9.80665 * sin(pivot.positionSupplier.get()) - pivot.velocitySupplier.get()
        .pow(2) * (ElevatorConstants.BASE_PIVOT_TO_CG_M + pivot.positionSupplier.get() * 0.75)
      ) / 9.80665 + this.kv * velocitySetpoint
  }

  companion object {
    fun createElevatorFeedForward(
      pivot: Pivot
    ): ElevatorFeedForward {
      return ElevatorFeedForward(
        pivot,
        ElevatorConstants.KS,
        ElevatorConstants.KV,
        ElevatorConstants.KG
      )
    }
  }
}
