package frc.team449.subsystems.superstructure.elevator

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.NumericalIntegration
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.simulation.ElevatorSim
import kotlin.math.sin

class TiltedElevatorSim(
  gearbox: DCMotor,
  gearing: Double,
  carriageMass: Double,
  drumRadius: Double,
  private val minHeight: Double,
  private val maxHeight: Double,
  private val simulateGravity: Boolean = true,
  private var angle: Double
) : ElevatorSim(gearbox, gearing, carriageMass, drumRadius, minHeight, maxHeight, simulateGravity, 0.0) {

  fun changeAngle(newAngle: Double) {
    this.angle = newAngle
  }

  override fun updateX(currentXhat: Matrix<N2?, N1>, u: Matrix<N1?, N1>, dtSeconds: Double): Matrix<N2?, N1> {
    // Calculate updated x-hat from Runge-Kutta.
    val updatedXhat = NumericalIntegration.rkdp(
      { x: Matrix<N2?, N1>?, _u: Matrix<N1?, N1>? ->
        var xdot = m_plant.a.times(x).plus(m_plant.b.times(_u))
        if (simulateGravity) {
          xdot = xdot.plus(
            VecBuilder.fill(
              0.0,
              -9.81 * sin(
                Units.degreesToRadians(angle)
              )
            )
          )
        }
        xdot
      },
      currentXhat,
      u,
      dtSeconds
    )

    // We check for collisions after updating x-hat.

    // We check for collisions after updating x-hat.
    if (wouldHitLowerLimit(updatedXhat[0, 0])) {
      return VecBuilder.fill(minHeight, 0.0)
    }
    return if (wouldHitUpperLimit(updatedXhat[0, 0])) {
      VecBuilder.fill(maxHeight, 0.0)
    } else {
      updatedXhat!!
    }
  }
}
