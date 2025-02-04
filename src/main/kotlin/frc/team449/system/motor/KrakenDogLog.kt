package frc.team449.system.motor

import com.ctre.phoenix6.hardware.TalonFX
import dev.doglog.DogLog

object KrakenDogLog : DogLog() {

  fun log(key: String, motor: TalonFX) {
    log("$key/StatorCurrent", motor.statorCurrent.valueAsDouble)
    log("$key/SupplyCurrent", motor.supplyCurrent.valueAsDouble)
    log("$key/Position", motor.position.valueAsDouble)
    log("$key/Velocity", motor.velocity.valueAsDouble)
    log("$key/Voltage", motor.motorVoltage.valueAsDouble)
  }
}
