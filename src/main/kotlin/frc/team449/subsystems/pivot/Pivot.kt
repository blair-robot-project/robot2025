package frc.team449.subsystems.pivot

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.subsystems.superstructure.SuperstructureGoal
import java.util.function.Supplier
import kotlin.math.abs

class Pivot(
  private val motor: TalonFX
) : SubsystemBase() {

  val positionSupplier = Supplier { motor.position.valueAsDouble }
  val velocitySupplier = Supplier { motor.velocity.valueAsDouble }
  val targetSupplier = Supplier { request.Position }

  lateinit var pivotFeedForward: PivotFeedForward

//  val pivotSim = LengthPivotSim(
//    DCMotor.getKrakenX60(1),
//    1 / PivotConstants.GEARING,
//    PivotConstants.MOMENT_OF_INERTIA,
//    PivotConstants.ARM_LENGTH,
//    PivotConstants.MIN_ANGLE,
//    PivotConstants.MAX_ANGLE,
//    false,
//    PivotConstants.MIN_ANGLE
//  )
//
//  val simPositionSupplier = Supplier { pivotSim.angleRads }

  private val request: MotionMagicVoltage = MotionMagicVoltage(
    SuperstructureGoal.STOW.pivot.`in`(Radians)
  )

  // last request is sticky
  fun setPosition(position: Double): Command {
    return this.run {
      motor.setControl(
        request
          .withPosition(position)
          .withFeedForward(pivotFeedForward.calculateWithLength(motor.closedLoopReference.valueAsDouble, motor.closedLoopReferenceSlope.valueAsDouble))
      )
    }.until(::atSetpoint)
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
    return (abs(positionSupplier.get() - request.Position) < PivotConstants.TOLERANCE)
  }

  override fun periodic() {}

  override fun simulationPeriodic() {
    val motorSimState = motor.simState

    motorSimState.setRawRotorPosition(request.Position / (PivotConstants.GEARING * PivotConstants.UPR))
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Pivot Info")
    builder.addDoubleProperty("1.1 Voltage", { motor.motorVoltage.valueAsDouble }, null)
    builder.addDoubleProperty("1.2 Position", { positionSupplier.get() }, null)
    builder.addDoubleProperty("1.3 Velocity", { velocitySupplier.get() }, null)
    builder.addDoubleProperty("1.4 Desired Position", { request.Position }, null)
    builder.addBooleanProperty("1.5 At Tolerance", { atSetpoint() }, null)
    // builder.addStringProperty("1.7 Command", {this.currentCommand.name}, null)
  }

  companion object {
    fun createPivot(): Pivot {
      val leadMotor = TalonFX(PivotConstants.MOTOR_ID)
      val config = TalonFXConfiguration()

      config.MotorOutput.Inverted = PivotConstants.INVERTED
      config.MotorOutput.NeutralMode = PivotConstants.BRAKE_MODE
      config.MotorOutput.DutyCycleNeutralDeadband = 0.001
      config.Feedback.SensorToMechanismRatio = 1 / (PivotConstants.GEARING * PivotConstants.UPR)

      config.CurrentLimits.StatorCurrentLimitEnable = true
      config.CurrentLimits.SupplyCurrentLimitEnable = true
      config.CurrentLimits.StatorCurrentLimit = PivotConstants.STATOR_LIM
      config.CurrentLimits.SupplyCurrentLimit = PivotConstants.SUPPLY_LIM

      /** If we gonna have FOC in the future
       config.TorqueCurrent.PeakForwardTorqueCurrent = torqueCurrentLimit.`in`(Amps)
       config.TorqueCurrent.PeakReverseTorqueCurrent = -torqueCurrentLimit.`in`(Amps)
       **/

      config.Slot0.kP = PivotConstants.KP
      config.Slot0.kI = PivotConstants.KI
      config.Slot0.kD = PivotConstants.KD

      config.MotionMagic.MotionMagicCruiseVelocity = PivotConstants.CRUISE_VEL
      config.MotionMagic.MotionMagicAcceleration = PivotConstants.MAX_ACCEL

      val status1 = leadMotor.configurator.apply(config)
      if (!status1.isOK) println("Error applying configs to Pivot Motor -> Error Code: $status1")

      BaseStatusSignal.setUpdateFrequencyForAll(
        PivotConstants.VALUE_UPDATE_RATE,
        leadMotor.position,
        leadMotor.velocity,
        leadMotor.motorVoltage,
        leadMotor.supplyCurrent,
        leadMotor.statorCurrent,
        leadMotor.deviceTemp
      )

      return Pivot(leadMotor)
    }
  }
}
