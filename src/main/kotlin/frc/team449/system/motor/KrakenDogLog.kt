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
    log("$key/Closed Loop Pos", motor.closedLoopReference.valueAsDouble)
    log("$key/Closed Loop Vel", motor.closedLoopReferenceSlope.valueAsDouble)
    log("$key/Closed Loop Feedforward", motor.closedLoopFeedForward.valueAsDouble)
    log("$key/Closed Loop PID Output", motor.closedLoopOutput.valueAsDouble)
  }
}
