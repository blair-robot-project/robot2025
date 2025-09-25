package frc.team449.subsystems.superstructure.climb
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.subsystems.superstructure.intake.IntakeConstants
import frc.team449.system.motor.KrakenDogLog

// TODO(the entire class bru)
class Climb(
  private val motor: TalonFX,
//  private val sensor: DigitalInput
) : SubsystemBase() {
  fun runClimbWheels(): Command =
    runOnce {
      motor.setVoltage(ClimbConstants.RUN_VOLTAGE)
    }

//  fun cageDetected(): Boolean {
//    return !sensor.get()
//  }

  fun waitUntilCurrentSpike(): Command =
    WaitCommand(0.1750)
//      .andThen(WaitUntilCommand { motor.statorCurrent.valueAsDouble > 40.0 })
      .andThen(WaitUntilCommand { climbDebouncer.calculate(motor.statorCurrent.valueAsDouble > 40) })

  private val climbDebouncer = Debouncer(ClimbConstants.CLIMB_DEBOUNCER_WAIT, Debouncer.DebounceType.kRising)

  fun stop(): Command =
    runOnce {
      motor.stopMotor()
    }

  override fun periodic() {
    logData()
  }

  private fun logData() {
    KrakenDogLog.log("Climb/Climb Motor", motor)
  }

  companion object {
    fun createClimb(): Climb {
      val motor = TalonFX(ClimbConstants.MOTOR_ID)
      val config = TalonFXConfiguration()

      config.MotorOutput.Inverted = ClimbConstants.INVERTED
      config.MotorOutput.NeutralMode = ClimbConstants.BRAKE_MODE

      config.CurrentLimits.StatorCurrentLimitEnable = true
      config.CurrentLimits.SupplyCurrentLimitEnable = true
      config.CurrentLimits.SupplyCurrentLimit = ClimbConstants.SUPPLY_LIM
      config.CurrentLimits.StatorCurrentLimit = ClimbConstants.STATOR_LIM
      motor.configurator.apply(config)

//      BaseStatusSignal.setUpdateFrequencyForAll(
//        10,
//        motor.position,
//        motor.velocity,
//        motor.motorVoltage,
//        motor.closedLoopReference,
//        motor.closedLoopReferenceSlope,
//        motor.closedLoopFeedForward,
//        motor.deviceTemp
//      )

      return Climb(motor)
    }
  }
}
