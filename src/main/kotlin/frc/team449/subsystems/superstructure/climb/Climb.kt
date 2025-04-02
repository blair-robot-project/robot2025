package frc.team449.subsystems.superstructure.climb

import com.revrobotics.spark.SparkMax
import dev.doglog.DogLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.system.motor.createSparkMax

// TODO(the entire class bru)
class Climb(
  private val motor: SparkMax
//  private val sensor: DigitalInput
) : SubsystemBase() {

  fun runClimbWheels(): Command {
    return runOnce {
      motor.setVoltage(ClimbConstants.RUN_VOLTAGE)
    }
  }

  fun holdClimbWheels(): Command {
    return runOnce {
      motor.setVoltage(ClimbConstants.HOLD_VOLTAGE)
    }
  }

//  fun cageDetected(): Boolean {
//    return !sensor.get()
//  }

  fun waitUntilCurrrentSpike(): Command {
    return WaitCommand(0.1750)
      .andThen(WaitUntilCommand { motor.outputCurrent > 22.5 })
  }

  fun stop(): Command {
    return runOnce {
      motor.stopMotor()
    }
  }

  override fun periodic() {
    logData()
  }

  private fun logData() {
    DogLog.log("Climb/Motor Voltage", motor.appliedOutput * 12.0)
    DogLog.log("Climb/Motor Current", motor.outputCurrent)
  }

  companion object {
    fun createClimb(): Climb {
      val motor = createSparkMax(
        id = ClimbConstants.MOTOR_ID,
        inverted = ClimbConstants.INVERTED,
        brakeMode = ClimbConstants.BRAKE_MODE,
        currentLimit = ClimbConstants.CURRENT_LIMIT
      )

//      val sensor = DigitalInput(ClimbConstants.SENSOR_DIO_PORT)

      return Climb(motor) // , sensor)
    }
  }
}
