package frc.team449.system.motor

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.REVLibError
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Frequency
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.RobotController

fun createSparkMax(
  id: Int,
  inverted: Boolean,
  brakeMode: Boolean = true,
  gearing: Double = 1.0,
  upr: Double = 1.0,
  controlPeriod: Time = Milliseconds.of(0.0),
  currentLimit: Current = Amps.of(40.0)
): SparkMax {
  val motor = SparkMax(id, SparkLowLevel.MotorType.kBrushless)
  if (motor.lastError != REVLibError.kOk) {
    println("Motor could not be constructed on port $id due to error ${motor.lastError}")
  }

  val config = SparkMaxConfig()

  config
    .inverted(inverted)
    .idleMode(if (brakeMode) IdleMode.kBrake else IdleMode.kCoast)
    .smartCurrentLimit(
      currentLimit.`in`(Amps).toInt(),
    )
    .voltageCompensation(RobotController.getBatteryVoltage())

  config.encoder
    .positionConversionFactor(upr * gearing)
    .velocityConversionFactor(upr * gearing * 1 / 60)

  motor.configure(
    config,
    SparkBase.ResetMode.kResetSafeParameters,
    SparkBase.PersistMode.kPersistParameters
  )

  motor.setControlFramePeriodMs(controlPeriod.`in`(Milliseconds).toInt())
  motor.setPeriodicFrameTimeout(0) // sets to minimum (2.1 * period)

  return motor
}

fun createFollowerSpark(
  id: Int,
  leader: SparkMax,
  invertedFromLeader: Boolean
): SparkMax {
  val follower = SparkMax(id, SparkLowLevel.MotorType.kBrushless)
  val config = SparkMaxConfig()
  val leaderConfig = leader.configAccessor

  config
    .inverted(leaderConfig.inverted)
    .idleMode(leaderConfig.idleMode)
    .smartCurrentLimit(
      leaderConfig.smartCurrentLimit,
      leaderConfig.smartCurrentFreeLimit
    )
    .voltageCompensation(leaderConfig.voltageCompensation)
    .follow(leader, invertedFromLeader)

  config.encoder
    .positionConversionFactor(leaderConfig.encoder.positionConversionFactor)
    .velocityConversionFactor(leaderConfig.encoder.velocityConversionFactor)

  follower.configure(
    config,
    SparkBase.ResetMode.kResetSafeParameters,
    SparkBase.PersistMode.kPersistParameters
  )

  return follower
}

fun createKraken(
  id: Int,
  inverted: Boolean,
  brakeMode: Boolean = true,
  sensorToMech: Double = 1.0,
  dutyCycleDeadband: Double = 0.001,
  statorCurrentLimit: Current = Amps.of(150.0),
  burstCurrentLimit: Current = Amps.of(60.0),
  burstTimeLimit: Time = Seconds.of(0.25),
  supplyCurrentLimit: Current = Amps.of(40.0),
  torqueCurrentLimit: Current = Amps.of(60.0),
  kS: Double = 0.0,
  kV: Double = 0.0,
  kA: Double = 0.0,
  kG: Double = 0.0,
  kP: Double = 0.0,
  kI: Double = 0.0,
  kD: Double = 0.0,
  gravityType: GravityTypeValue = GravityTypeValue.Elevator_Static,
  cruiseVel: Double = 0.0,
  maxAccel: Double = 0.0,
  maxJerk: Double = Double.NaN,
  updateFrequency: Frequency = Hertz.of(50.0)
): TalonFX {
  val motor = TalonFX(id)
  val config = TalonFXConfiguration()

  config.MotorOutput.Inverted = if (inverted) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive
  config.MotorOutput.NeutralMode = if (brakeMode) NeutralModeValue.Brake else NeutralModeValue.Coast
  config.MotorOutput.DutyCycleNeutralDeadband = dutyCycleDeadband
  config.Feedback.SensorToMechanismRatio = sensorToMech

  config.CurrentLimits.StatorCurrentLimitEnable = true
  config.CurrentLimits.SupplyCurrentLimitEnable = true
  config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit.`in`(Amps)
  config.CurrentLimits.SupplyCurrentLowerLimit = supplyCurrentLimit.`in`(Amps)
  config.CurrentLimits.SupplyCurrentLimit = burstCurrentLimit.`in`(Amps)
  config.CurrentLimits.SupplyCurrentLowerTime = burstTimeLimit.`in`(Seconds)
  config.TorqueCurrent.PeakForwardTorqueCurrent = torqueCurrentLimit.`in`(Amps)
  config.TorqueCurrent.PeakReverseTorqueCurrent = -torqueCurrentLimit.`in`(Amps)

  config.Slot0.kS = kS
  config.Slot0.kV = kV
  config.Slot0.kA = kA
  config.Slot0.kG = kG
  config.Slot0.kP = kP
  config.Slot0.kI = kI
  config.Slot0.kD = kD

  config.Slot0.GravityType = gravityType

  config.MotionMagic.MotionMagicCruiseVelocity = cruiseVel
  config.MotionMagic.MotionMagicAcceleration = maxAccel
  if (!maxJerk.isNaN()) {
    config.MotionMagic.MotionMagicJerk = maxJerk
  }

  val status = motor.configurator.apply(config)
  if (!status.isOK) println("Could not apply configs, error code: $status")

  BaseStatusSignal.setUpdateFrequencyForAll(
    updateFrequency,
    motor.position,
    motor.velocity,
    motor.motorVoltage,
    motor.supplyCurrent,
    motor.statorCurrent,
    motor.deviceTemp
  )

  motor.optimizeBusUtilization()

  return motor
}
