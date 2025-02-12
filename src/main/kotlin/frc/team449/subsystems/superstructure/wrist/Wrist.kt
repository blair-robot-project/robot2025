package frc.team449.subsystems.superstructure.wrist

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Robot
import frc.team449.subsystems.superstructure.SuperstructureGoal
import java.util.function.Supplier
import kotlin.math.abs

// TODO(the entire class bru)
class Wrist(
  private val motor: TalonFX
) : SubsystemBase() {

  val positionSupplier = Supplier { motor.position.valueAsDouble }
  val velocitySupplier = Supplier { motor.velocity.valueAsDouble }
  val targetSupplier = Supplier { request.Position }

  private val request = MotionMagicVoltage(
    SuperstructureGoal.STOW.wrist.`in`(Radians)
  )

  fun setPosition(position: Double): Command {
    return this.runOnce {
      motor.setControl(
        request
          .withPosition(position)
      )
    } // .until(::atSetpoint)
  }

  fun manualDown(): Command {
    return runOnce { motor.setVoltage(-3.0) }
  }

  fun manualUp(): Command {
    return runOnce { motor.setVoltage(3.0) }
  }

  fun stop(): Command {
    return this.runOnce { motor.stopMotor() }
  }

  private fun atSetpoint(): Boolean {
    return (abs(positionSupplier.get() - request.Position) < WristConstants.TOLERANCE)
  }

  override fun periodic() {}

  override fun simulationPeriodic() {
    val motorSimState = motor.simState

    motorSimState.setRawRotorPosition(request.Position / (WristConstants.GEARING * WristConstants.UPR))
  }

  companion object {
    fun createWrist(
      robot: Robot
    ): Wrist {
      val leadMotor = TalonFX(WristConstants.MOTOR_ID)
      val config = TalonFXConfiguration()

      config.MotorOutput.Inverted = WristConstants.INVERTED
      config.MotorOutput.NeutralMode = WristConstants.BRAKE_MODE
      config.MotorOutput.DutyCycleNeutralDeadband = 0.001
      config.Feedback.SensorToMechanismRatio = 1 / (WristConstants.GEARING * WristConstants.UPR)

      config.CurrentLimits.StatorCurrentLimitEnable = true
      config.CurrentLimits.SupplyCurrentLimitEnable = true
      config.CurrentLimits.StatorCurrentLimit = WristConstants.STATOR_LIM
      config.CurrentLimits.SupplyCurrentLimit = robot.currentManager.wristSUPPLY_LIM()

      /** If we gonna have FOC in the future
       config.TorqueCurrent.PeakForwardTorqueCurrent = torqueCurrentLimit.`in`(Amps)
       config.TorqueCurrent.PeakReverseTorqueCurrent = -torqueCurrentLimit.`in`(Amps)
       **/

      config.Slot0.kP = WristConstants.KP
      config.Slot0.kI = WristConstants.KI
      config.Slot0.kD = WristConstants.KD

      config.MotionMagic.MotionMagicCruiseVelocity = WristConstants.CRUISE_VEL
      config.MotionMagic.MotionMagicAcceleration = WristConstants.MAX_ACCEL

      val status1 = leadMotor.configurator.apply(config)
      if (!status1.isOK) println("Error applying configs to Wrist Motor -> Error Code: $status1")

      BaseStatusSignal.setUpdateFrequencyForAll(
        WristConstants.VALUE_UPDATE_RATE,
        leadMotor.position,
        leadMotor.velocity,
        leadMotor.motorVoltage,
        leadMotor.supplyCurrent,
        leadMotor.statorCurrent,
        leadMotor.deviceTemp
      )

      return Wrist(leadMotor)
    }
  }
}
