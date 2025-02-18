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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.subsystems.superstructure.SuperstructureGoal
import frc.team449.system.motor.KrakenDogLog
import java.util.function.Supplier
import kotlin.math.abs

// TODO(the entire class bru)
class Wrist(
  private val motor: TalonFX
//  val absoluteEncoder: AbsoluteEncoder,
//  val quadEncoder: QuadEncoder
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
    }.andThen(
      WaitUntilCommand(::atSetpoint)
    )
  }

  fun hold(): Command {
    return this.runOnce {
      motor.setControl(
        request
          .withPosition(request.Position)
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
    return runOnce {
      motor.setVoltage(1.0)
      request.Position = positionSupplier.get()
    }
  }

  fun stop(): Command {
    return this.runOnce { motor.stopMotor() }
  }

  private fun atSetpoint(): Boolean {
    return (abs(positionSupplier.get() - targetSupplier.get()) < WristConstants.TOLERANCE.`in`(Radians))
  }

  fun startupZero() {
    motor.setPosition(WristConstants.STARTUP_ANGLE.`in`(Radians))
  }

  override fun periodic() {
    logData()

    // No quad encoder
//    if (abs(motor.position.valueAsDouble - quadEncoder.position) > WristConstants.RESET_ENC_LIMIT.`in`(Radians) && isReal) {
//      motor.setPosition(quadEncoder.position)
//    }
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
/*    DogLog.log("Wrist/Abs/Pos", absoluteEncoder.position)
    DogLog.log("Wrist/Abs/Vel", absoluteEncoder.velocity)*/
//    No quad Encoder
//    DogLog.log("Wrist/Quad/Pos", quadEncoder.position)
//    DogLog.log("Wrist/Quad/Vel", quadEncoder.velocity)
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
        leadMotor.deviceTemp
      )

//      val absEnc = AbsoluteEncoder.createAbsoluteEncoder(
//        "Wrist Absolute Enc",
//        WristConstants.ABS_ENC_DIO_PORT,
//        WristConstants.ABS_OFFSET,
//        WristConstants.ENC_RATIO,
//        WristConstants.ENC_INVERTED,
//        min = WristConstants.ABS_RANGE.first,
//        max = WristConstants.ABS_RANGE.second
//      )

//      val quadEnc = QuadEncoder.createQuadEncoder(
//        "Wrist Quad Enc",
//        WristConstants.QUAD_ENCODER,
//        WristConstants.ENC_CPR,
//        WristConstants.ENC_RATIO,
//        1.0,
//        WristConstants.ENC_INVERTED,
//        WristConstants.SAMPLES_TO_AVERAGE
//      )

      return Wrist(leadMotor) // , absEnc)//, quadEnc)
    }
  }
}
