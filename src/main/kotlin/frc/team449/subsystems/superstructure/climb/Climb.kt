package frc.team449.subsystems.superstructure.climb

import com.revrobotics.spark.SparkMax
import dev.doglog.DogLog
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.createFollowerSpark
import frc.team449.system.motor.createSparkMax

// TODO(the entire class bru)
class Climb(
  private val motor: SparkMax,
  private val infrared: DigitalInput,
  private val infrared2: DigitalInput
) : SubsystemBase() {

  fun runClimbWheels(): Command {
    return runOnce {
      motor.setVoltage(ClimbConstants.RUN_VOLTAGE)
    }
  }

  fun stop(): Command {
    return runOnce {
      motor.stopMotor()
    }
  }

  fun isClimbEngaged(): Boolean {
    return !infrared.get() && !infrared2.get()
  }

  override fun periodic() {
    logData()
  }

  private fun logData() {
    DogLog.log("Climb/Motor Voltage", motor.appliedOutput * 12.0)
    DogLog.log("Climb/IR sensor 1", !infrared.get())
    DogLog.log("Climb/IR sensor 2", !infrared2.get())
  }

  companion object {
    fun createClimb(): Climb {
      val motor = createSparkMax(
        id = ClimbConstants.RIGHT_MOTOR_ID,
        inverted = ClimbConstants.RIGHT_INVERTED,
        brakeMode = ClimbConstants.BRAKE_MODE,
        currentLimit = ClimbConstants.CURRENT_LIMIT
      )

      createFollowerSpark(
        id = ClimbConstants.LEFT_MOTOR_ID,
        leader = motor,
        invertedFromLeader = ClimbConstants.LEFT_INVERTED_FROM_RIGHT
      )

      val sensor = DigitalInput(ClimbConstants.SENSOR_DIO_PORT)
      val sensor2 = DigitalInput(ClimbConstants.SENSOR2_DIO_PORT)

      return Climb(motor, sensor, sensor2)
    }
  }
}
