package frc.team449.system.encoder

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem

class QuadCalibration(
    subsystem: Subsystem,
    private val absolute: AbsoluteEncoder,
    private val encoder: QuadEncoder,
    private val numSamples: Int = 150,
) : Command() {

    init {
        addRequirements(subsystem)
    }

    private var samples = mutableListOf<Double>()

    override fun execute() {
        samples.add(absolute.position)
    }

    override fun isFinished(): Boolean {
        return samples.size >= numSamples
    }

    override fun end(interrupted: Boolean) {
        samples.sort()
        val angle = samples[(samples.size * .9).toInt()]
        encoder.resetPosition(angle)
        println("***** Finished Calibrating Quadrature reading *****")
    }
}
