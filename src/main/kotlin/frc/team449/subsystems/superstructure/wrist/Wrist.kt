package frc.team449.subsystems.superstructure.wrist

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.sim.ChassisReference
import dev.doglog.DogLog
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.subsystems.superstructure.SuperstructureGoal
import frc.team449.system.motor.KrakenDogLog
import java.util.function.Supplier
import kotlin.math.abs

class Wrist(
  private val motor: TalonFX
) : SubsystemBase() {

  val positionSupplier = Supplier { motor.position.valueAsDouble }
  val velocitySupplier = Supplier { motor.velocity.valueAsDouble }
  val targetSupplier = Supplier { request.Position }

  private val request = MotionMagicVoltage(
    SuperstructureGoal.STOW.wrist.`in`(Radians)
  ).withEnableFOC(false)

  private val isReal = RobotBase.isReal()

  lateinit var wristFeedForward: WristFeedForward

  fun setPosition(position: Double): Command {
    return this.runOnce {
      motor.setControl(
        request
          .withPosition(position)
          .withUpdateFreqHz(WristConstants.REQUEST_UPDATE_RATE)
          .withFeedForward(wristFeedForward.calculate(position))
      )
    }
  }

  fun hold(): Command {
    return this.runOnce {
      motor.setControl(
        request
          .withUpdateFreqHz(WristConstants.REQUEST_UPDATE_RATE)
          .withFeedForward(wristFeedForward.calculate(request.Position))
      )
    }
  }

  fun setVoltageChar(volts: Double) {
    motor.setControl(VoltageOut(volts))
  }

  fun manualDown(): Command {
    return run {
      motor.setVoltage(-1.0)
      request.Position = positionSupplier.get()
    }
  }

  fun manualUp(): Command {
    return run {
      motor.setVoltage(1.0)
      request.Position = positionSupplier.get()
    }
  }

  fun stop(): Command {
    return this.runOnce { motor.stopMotor() }
  }

  fun atSetpoint(): Boolean {
    return (abs(positionSupplier.get() - targetSupplier.get()) < WristConstants.TOLERANCE.`in`(Radians))
  }

  fun elevatorReady(): Boolean {
    return positionSupplier.get() < WristConstants.ELEVATOR_READY.`in`(Radians)
  }

  fun startupZero() {
    motor.setPosition(WristConstants.STARTUP_ANGLE.`in`(Radians))
  }

  override fun periodic() {
    logData()
  }

  override fun simulationPeriodic() {
    val motorSimState = motor.simState
    motorSimState.Orientation = ChassisReference.Clockwise_Positive
    motorSimState.setRawRotorPosition(motor.closedLoopReference.valueAsDouble / (WristConstants.GEARING * WristConstants.UPR))
  }

  private fun logData() {
    DogLog.log("Wrist/Desired Target", targetSupplier.get())
    DogLog.log("Wrist/Motion Magic Setpoint", motor.closedLoopReference.valueAsDouble)
    DogLog.log("Wrist/In Tolerance", atSetpoint())
    KrakenDogLog.log("Wrist/Motor", motor)
  }

  companion object {
    fun createWrist(): Wrist {
      val leadMotor = TalonFX(WristConstants.MOTOR_ID)
      val config = TalonFXConfiguration()

      config.MotorOutput.Inverted = WristConstants.INVERTED
      config.MotorOutput.NeutralMode = WristConstants.BRAKE_MODE
      config.MotorOutput.DutyCycleNeutralDeadband = 0.001
      config.Feedback.SensorToMechanismRatio = 1 / (WristConstants.GEARING * WristConstants.UPR)

      config.CurrentLimits.StatorCurrentLimitEnable = true
      config.CurrentLimits.SupplyCurrentLimitEnable = true
      config.CurrentLimits.StatorCurrentLimit = WristConstants.STATOR_LIM
      config.CurrentLimits.SupplyCurrentLimit = WristConstants.SUPPLY_LIM

      /** If we gonna have FOC in the future
       config.TorqueCurrent.PeakForwardTorqueCurrent = torqueCurrentLimit.`in`(Amps)
       config.TorqueCurrent.PeakReverseTorqueCurrent = -torqueCurrentLimit.`in`(Amps)
       **/

      config.Slot0.kP = WristConstants.KP
      config.Slot0.kI = WristConstants.KI
      config.Slot0.kD = WristConstants.KD

      config.Slot0.kS = WristConstants.KS
      config.Slot0.kV = WristConstants.KV

      config.MotionMagic.MotionMagicCruiseVelocity = WristConstants.CRUISE_VEL.`in`(RadiansPerSecond)
      config.MotionMagic.MotionMagicAcceleration = WristConstants.MAX_ACCEL.`in`(RadiansPerSecondPerSecond)

      val status1 = leadMotor.configurator.apply(config)
      if (!status1.isOK) println("Error applying configs to Wrist Motor -> Error Code: $status1")

      BaseStatusSignal.setUpdateFrequencyForAll(
        WristConstants.VALUE_UPDATE_RATE,
        leadMotor.position,
        leadMotor.velocity,
        leadMotor.motorVoltage,
        leadMotor.supplyCurrent,
        leadMotor.statorCurrent,
        leadMotor.closedLoopReference,
        leadMotor.closedLoopReferenceSlope,
        leadMotor.closedLoopFeedForward,
        leadMotor.deviceTemp
      )

      return Wrist(leadMotor)
    }
  }
}
