package frc.team449.subsystems.superstructure.pivot

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import dev.doglog.DogLog
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.subsystems.superstructure.SuperstructureGoal
import frc.team449.system.encoder.AbsoluteEncoder
import frc.team449.system.encoder.QuadEncoder
import frc.team449.system.motor.KrakenDogLog
import java.util.function.Supplier
import kotlin.math.abs

class Pivot(
  private val motor: TalonFX,
  val absoluteEncoder: AbsoluteEncoder,
  val quadEncoder: QuadEncoder
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

  private val isReal = RobotBase.isReal()

  // last request is sticky
  fun setPosition(position: Double): Command {
    return this.run {
      motor.setControl(
        request
          .withPosition(position)
          .withUpdateFreqHz(PivotConstants.REQUEST_UPDATE_RATE)
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

  fun setVoltage(voltage: Double): Command {
    return runOnce { motor.setVoltage(voltage) }
  }

  fun stop(): Command {
    return this.runOnce { motor.stopMotor() }
  }

  private fun atSetpoint(): Boolean {
    return (abs(positionSupplier.get() - request.Position) < PivotConstants.TOLERANCE)
  }

  override fun periodic() {
    logData()

    if (abs(motor.position.valueAsDouble - quadEncoder.position) > PivotConstants.RESET_ENC_LIMIT.`in`(Radians) && isReal) {
      motor.setPosition(quadEncoder.position)
    }
  }

  override fun simulationPeriodic() {
    val motorSimState = motor.simState

    motorSimState.setRawRotorPosition(motor.closedLoopReference.valueAsDouble / (PivotConstants.GEARING * PivotConstants.UPR))
  }

  private fun logData() {
    DogLog.log("Pivot/Desired Target", request.Position)
    DogLog.log("Pivot/Motion Magic Setpoint", motor.closedLoopReference.valueAsDouble)
    DogLog.log("Pivot/In Tolerance", atSetpoint())
    DogLog.log("Pivot/Abs/Pos", absoluteEncoder.position)
    DogLog.log("Pivot/Abs/Vel", absoluteEncoder.velocity)
    DogLog.log("Pivot/Quad/Pos", quadEncoder.position)
    DogLog.log("Pivot/Quad/Vel", quadEncoder.velocity)
    KrakenDogLog.log("Pivot/Motor", motor)
  }

  companion object {
    fun createPivot(): Pivot {
      val leadMotor = TalonFX(PivotConstants.LEAD_MOTOR_ID)
      val followerMotor = TalonFX(PivotConstants.FOLLOWER_MOTOR_ID)

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

      config.MotionMagic.MotionMagicCruiseVelocity = PivotConstants.CRUISE_VEL.`in`(RadiansPerSecond)
      config.MotionMagic.MotionMagicAcceleration = PivotConstants.MAX_ACCEL.`in`(RadiansPerSecondPerSecond)

      val status1 = leadMotor.configurator.apply(config)
      if (!status1.isOK) println("Error applying configs to Elevator Lead Motor -> Error Code: $status1")

      val status2 = followerMotor.configurator.apply(config)
      if (!status2.isOK) println("Error applying configs to Elevator Follower Motor -> Error Code: $status2")

      BaseStatusSignal.setUpdateFrequencyForAll(
        PivotConstants.VALUE_UPDATE_RATE,
        leadMotor.position,
        leadMotor.velocity,
        leadMotor.motorVoltage,
        leadMotor.supplyCurrent,
        leadMotor.statorCurrent,
        leadMotor.deviceTemp
      )

      leadMotor.optimizeBusUtilization()

      BaseStatusSignal.setUpdateFrequencyForAll(
        PivotConstants.VALUE_UPDATE_RATE,
        followerMotor.position,
        followerMotor.velocity,
        followerMotor.motorVoltage,
        followerMotor.supplyCurrent,
        followerMotor.statorCurrent,
        followerMotor.deviceTemp
      )

      followerMotor.optimizeBusUtilization()

      followerMotor.setControl(
        Follower(PivotConstants.LEAD_MOTOR_ID, PivotConstants.FOLLOWER_INVERTED_TO_MASTER)
      )

      val absEnc = AbsoluteEncoder.createAbsoluteEncoder(
        "Pivot Absolute Enc",
        PivotConstants.ABS_ENC_DIO_PORT,
        PivotConstants.ABS_OFFSET,
        PivotConstants.ENC_RATIO,
        PivotConstants.ENC_INVERTED,
        min = PivotConstants.ABS_RANGE.first,
        max = PivotConstants.ABS_RANGE.second
      )

      val quadEnc = QuadEncoder.createQuadEncoder(
        "Pivot Quad Enc",
        PivotConstants.QUAD_ENCODER,
        PivotConstants.ENC_CPR,
        PivotConstants.ENC_RATIO,
        1.0,
        PivotConstants.ENC_INVERTED,
        PivotConstants.SAMPLES_TO_AVERAGE
      )

      return Pivot(leadMotor, absEnc, quadEnc)
    }
  }
}
