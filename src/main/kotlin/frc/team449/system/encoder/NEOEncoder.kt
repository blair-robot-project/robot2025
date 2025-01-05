package frc.team449.system.encoder

import com.revrobotics.RelativeEncoder
import com.revrobotics.spark.SparkMax

/** A NEO integrated encoder plugged into a Spark */
class NEOEncoder(
  name: String,
  private val enc: RelativeEncoder,
  unitPerRotation: Double,
  gearing: Double,
  pollTime: Double = .01
) : Encoder(name, NEO_ENCODER_CPR, 1.0, 1.0, pollTime) {
  val positionConversionFactor = unitPerRotation * gearing
  val velocityConversionFactor = unitPerRotation * gearing / 60

  init {
    // Let the underlying encoder do the conversions
    // Divide by 60 because it's originally in RPM
  }

  override fun getPositionNative() = enc.position

  override fun pollVelocityNative(): Double = enc.velocity

  companion object {
    const val NEO_ENCODER_CPR = 1

    fun creator(
      unitPerRotation: Double,
      gearing: Double
    ): EncoderCreator<SparkMax> = EncoderCreator { name, motor, _ ->
      NEOEncoder(name, motor.encoder, unitPerRotation, gearing)
    }
  }
}
