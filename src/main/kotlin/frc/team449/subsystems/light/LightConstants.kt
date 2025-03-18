package frc.team449.subsystems.light

object LightConstants {
  const val LIGHT_PORT: Int = 9
  const val LIGHT_LENGTH: Int = 24

  const val SECTION_1_START = 0
  const val SECTION_1_END = 11
  const val SECTION_2_START = 12
  const val SECTION_2_END = 23

  // number of LEDs per meter (for WPILib calc)
  // todo(buscalo este numero)
  const val LED_PER_METER: Int = 120
  const val LED_TRANSLATION_SPEED: Double = 0.5 // m/s
}
