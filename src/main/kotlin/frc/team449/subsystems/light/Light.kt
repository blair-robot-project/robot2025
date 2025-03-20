package frc.team449.subsystems.light

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.LEDPattern.GradientType
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.function.DoubleSupplier

/**
 * Controls an LED strip.
 * @param port The PWM port of the LED strip.
 * @param length The length of the LED strip.
 */

class Light(
  port: Int,
  length: Int
) : SubsystemBase() {

  private val lightStrip = AddressableLED(port)
  private val lightBuffer = AddressableLEDBuffer(length)

  val ledSpacing: Distance = Meters.of(1.0 / LightConstants.LED_PER_METER)

  init {
    lightStrip.setLength(lightBuffer.length)
    lightStrip.setData(lightBuffer)
    lightStrip.start()
  }

  fun rainbow(): Command {
    val rainbow: LEDPattern = LEDPattern.rainbow(255, 128)
    val scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(
      MetersPerSecond.of(LightConstants.LED_TRANSLATION_SPEED),
      ledSpacing
    )

    return this.run { scrollingRainbow.applyTo(lightBuffer) }
      .ignoringDisable(true)
  }

  /** Some LED strips will encode their data into GRB instead
   * of RGB, which will not work with the provided colors
   * in WPILib's Color's class.
   */
  fun changeForGRB(color: Color): Color {
    return Color(color.green, color.red, color.blue)
  }

  fun off(): Command {
    return this.run { LEDPattern.kOff.applyTo(lightBuffer) }
      .ignoringDisable(true)
  }

  fun solidColor(color: Color): Command {
    val colorSolid: LEDPattern = LEDPattern.solid(color)

    return this.run { colorSolid.applyTo(lightBuffer) }
      .ignoringDisable(true)
  }

  fun blink(time: Time, color: Color): Command {
    val colorPattern = LEDPattern.solid(color)
    val blinking = colorPattern.blink(time)

    return this.run { blinking.applyTo(lightBuffer) }
      .ignoringDisable(true)
  }

  fun gradient(speed: LinearVelocity = MetersPerSecond.of(0.0), vararg colors: Color): Command {
    // continuous is optimal for scrolling, discontinuous for static
    val continuousness: GradientType = if (speed.baseUnitMagnitude() != 0.0) GradientType.kContinuous else GradientType.kDiscontinuous
    val gradient: LEDPattern = LEDPattern.gradient(continuousness, *colors)
    val pattern = gradient.scrollAtAbsoluteSpeed(speed, ledSpacing)

    return this.run { pattern.applyTo(lightBuffer) }
      .ignoringDisable(true)
  }

  fun breath(time: Time, color: Color): Command {
    val solid: LEDPattern = LEDPattern.solid(color)
    val pattern = solid.breathe(time)

    return this.run { pattern.applyTo(lightBuffer) }
      .ignoringDisable(true)
  }

  fun progressMaskGradient(maskValue: DoubleSupplier): Command {
    val base: LEDPattern = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlue, Color.kBlueViolet, Color.kDeepPink, Color.kIndianRed)
    val maskAmount: LEDPattern = LEDPattern.progressMaskLayer(maskValue)
    val heightDisplay: LEDPattern = base.mask(maskAmount)

    return this.run { heightDisplay.applyTo(lightBuffer) }
      .ignoringDisable(true)
  }

  override fun periodic() {
    lightStrip.setData(lightBuffer)
  }

  companion object {
    /** Create an LED strip controller using [LightConstants]. */
    fun createLight(): Light {
      return Light(
        LightConstants.LIGHT_PORT,
        LightConstants.LIGHT_LENGTH
      )
    }
  }
}
