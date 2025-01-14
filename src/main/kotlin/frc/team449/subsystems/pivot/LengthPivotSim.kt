package frc.team449.subsystems.pivot

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.NumericalIntegration
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.LinearSystemSim
import kotlin.math.cos
import kotlin.math.sign

class LengthPivotSim(
    plant: LinearSystem<N2?, N1?, N2?>?,
    private val m_gearbox: DCMotor,
    private val m_gearing: Double,
    var m_armLenMeters: Double,
    private val m_minAngle: Double,
    private val m_maxAngle: Double,
    private val m_simulateGravity: Boolean,
    startingAngleRads: Double,
    vararg measurementStdDevs: Double,
) : LinearSystemSim<N2?, N1?, N2?>(plant, *measurementStdDevs) {
    init {
        this.setState(startingAngleRads, 0.0)
    }

    constructor(
        gearbox: DCMotor,
        gearing: Double,
        jKgMetersSquared: Double,
        armLengthMeters: Double,
        minAngleRads: Double,
        maxAngleRads: Double,
        simulateGravity: Boolean,
        startingAngleRads: Double,
        vararg measurementStdDevs: Double,
    ) : this(
        LinearSystemId.createSingleJointedArmSystem(gearbox, jKgMetersSquared, gearing),
        gearbox,
        gearing,
        armLengthMeters,
        minAngleRads,
        maxAngleRads,
        simulateGravity,
        startingAngleRads,
        *measurementStdDevs,
    )

    fun changeArmLength(newArmLengthMeters: Double) {
        this.m_armLenMeters = newArmLengthMeters
    }

    fun setState(angleRadians: Double, velocityRadPerSec: Double) {
        this.setState(
            VecBuilder.fill(
                MathUtil.clamp(angleRadians, this.m_minAngle, this.m_maxAngle),
                velocityRadPerSec,
            )
        )
    }

    fun wouldHitLowerLimit(currentAngleRads: Double): Boolean {
        return currentAngleRads <= this.m_minAngle
    }

    fun wouldHitUpperLimit(currentAngleRads: Double): Boolean {
        return currentAngleRads >= this.m_maxAngle
    }

    fun hasHitLowerLimit(): Boolean {
        return this.wouldHitLowerLimit(this.angleRads)
    }

    fun hasHitUpperLimit(): Boolean {
        return this.wouldHitUpperLimit(this.angleRads)
    }

    val angleRads: Double
        get() = this.getOutput(0)

    val velocityRadPerSec: Double
        get() = this.getOutput(1)

    val currentDrawAmps: Double
        get() {
            val motorVelocity = m_x[1, 0] * this.m_gearing
            return m_gearbox.getCurrent(motorVelocity, m_u[0, 0]) * sign(m_u[0, 0])
        }

    fun setInputVoltage(volts: Double) {
        this.setInput(*doubleArrayOf(volts))
        this.clampInput(RobotController.getBatteryVoltage())
    }

    override fun updateX(
        currentXhat: Matrix<N2?, N1>,
        u: Matrix<N1?, N1>,
        dtSeconds: Double,
    ): Matrix<N2?, N1> {
        val updatedXhat =
            NumericalIntegration.rkdp(
                { x: Matrix<N2?, N1>, _u: Matrix<N1?, N1>? ->
                    var xdot = m_plant.a.times(x).plus(m_plant.b.times(_u))
                    if (this.m_simulateGravity) {
                        val alphaGrav = -14.700000000000001 * cos(x[0, 0]) / this.m_armLenMeters
                        xdot = xdot.plus(VecBuilder.fill(0.0, alphaGrav))
                    }
                    xdot
                },
                currentXhat,
                u,
                dtSeconds,
            )
        return if (this.wouldHitLowerLimit(updatedXhat[0, 0])) {
            VecBuilder.fill(this.m_minAngle, 0.0)
        } else {
            (if (this.wouldHitUpperLimit(updatedXhat[0, 0])) VecBuilder.fill(this.m_maxAngle, 0.0)
            else updatedXhat)
                as Matrix<N2?, N1>
        }
    }

    companion object {
        fun estimateMOI(lengthMeters: Double, massKg: Double): Double {
            return 0.3333333333333333 * massKg * lengthMeters * lengthMeters
        }
    }
}
