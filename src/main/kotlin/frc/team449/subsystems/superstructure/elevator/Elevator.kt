package frc.team449.subsystems.superstructure.elevator

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import dev.doglog.DogLog
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.subsystems.superstructure.SuperstructureGoal
import frc.team449.subsystems.superstructure.wrist.WristConstants
import frc.team449.system.motor.KrakenDogLog
import java.util.function.Supplier
import kotlin.math.abs

open class Elevator(
  private val motor: TalonFX
) : SubsystemBase() {

  open val positionSupplier = Supplier { motor.position.valueAsDouble }
  open val velocitySupplier = Supplier { motor.velocity.valueAsDouble }
  open val targetSupplier = Supplier { motor.closedLoopReference.valueAsDouble }
  open val goalSupplier = Supplier { request.Position }

  lateinit var elevatorFeedForward: ElevatorFeedForward

  open val elevatorSim: TiltedElevatorSim? = null

  val mech: Mechanism2d = Mechanism2d(3.0, 3.0, Color8Bit(0, 0, 0))
  private val rootElevator: MechanismRoot2d = mech.getRoot("elevatorRoot", 0.25, 0.25)
  val elevatorLigament: MechanismLigament2d = rootElevator.append(
    MechanismLigament2d(
      "elevatorLigament",
      ElevatorConstants.SIM_MIN_HEIGHT,
      ElevatorConstants.ANGLE,
      ElevatorConstants.WIDTH,
      ElevatorConstants.COLOR
    )
  )

  val desiredElevatorLigament: MechanismLigament2d = rootElevator.append(
    MechanismLigament2d(
      "elevatorDesiredLigament",
      ElevatorConstants.SIM_MIN_HEIGHT,
      ElevatorConstants.ANGLE,
      ElevatorConstants.DESIRED_WIDTH,
      ElevatorConstants.DESIRED_COLOR
    )
  )

  val wristLigament: MechanismLigament2d = elevatorLigament.append(
    MechanismLigament2d(
      "wristLigament",
      0.2,
      WristConstants.ANGLE,
      WristConstants.WIDTH,
      WristConstants.COLOR
    )
  )

  private val request: MotionMagicVoltage = MotionMagicVoltage(
    SuperstructureGoal.STOW.elevator.`in`(Meters)
  ).withEnableFOC(false)

  fun setPosition(position: Double): Command {
    return this.runOnce {
      motor.setControl(
        request
          .withPosition(position)
          .withUpdateFreqHz(ElevatorConstants.REQUEST_UPDATE_RATE)
          .withFeedForward(elevatorFeedForward.calculateGravity())
      )
    }
  }

  fun setPositionCarriage(position: Double): Command {
    return this.runOnce {
      motor.setControl(
        request
          .withPosition(position)
          .withUpdateFreqHz(ElevatorConstants.REQUEST_UPDATE_RATE)
          .withFeedForward(ElevatorConstants.L4_FF)
          .withSlot(1)
      )
    }
  }

  fun manualDown(): Command {
    return this.run {
      motor.setVoltage(-2.0)
      request.Position = positionSupplier.get()
    }
  }

  fun manualUp(): Command {
    return this.run {
      motor.setVoltage(2.0)
      request.Position = positionSupplier.get()
    }
  }

  fun webComManualUp(voltage: Double): Command {
    return runOnce {
      motor.setVoltage(voltage)
      request.Position = positionSupplier.get()
    }
  }

  fun webComManualDown(voltage: Double): Command {
    return runOnce {
      motor.setVoltage(-voltage)
      request.Position = positionSupplier.get()
    }
  }

  fun hold(): Command {
    return this.runOnce {
      motor.setControl(
        PositionVoltage(request.Position)
          .withUpdateFreqHz(ElevatorConstants.REQUEST_UPDATE_RATE)
          .withFeedForward(elevatorFeedForward.calculateGravity())
      )
    }
  }

  fun holdCarriage(): Command {
    return this.runOnce {
      motor.setControl(
        PositionVoltage(request.Position)
          .withUpdateFreqHz(ElevatorConstants.REQUEST_UPDATE_RATE)
          .withFeedForward(ElevatorConstants.L4_FF)
          .withSlot(1)
      )
    }
  }

  fun setVoltage(voltage: Double) {
    return motor.setVoltage(voltage)
  }

  fun stop(): Command {
    return this.runOnce { motor.stopMotor() }
  }

  fun atSetpoint(): Boolean {
    return (abs(positionSupplier.get() - request.Position) < ElevatorConstants.TOLERANCE)
  }

  override fun periodic() {
    logData()
  }

  private fun logData() {
    DogLog.log("Elevator/Desired Target", request.Position)
    DogLog.log("Elevator/Motion Magic Setpoint", motor.closedLoopReference.valueAsDouble)
    DogLog.log("Elevator/In Tolerance", atSetpoint())
    KrakenDogLog.log("Elevator/Motor", motor)
  }

  companion object {
    fun createElevator(): Elevator {
      // TODO(Fill in parameters.)
      val leadMotor = TalonFX(ElevatorConstants.LEAD_MOTOR_ID)
      val followerMotor = TalonFX(ElevatorConstants.FOLLOWER_MOTOR_ID)
      val config = TalonFXConfiguration()

      config.MotorOutput.Inverted = ElevatorConstants.INVERTED
      config.MotorOutput.NeutralMode = ElevatorConstants.BRAKE_MODE
      config.MotorOutput.DutyCycleNeutralDeadband = 0.001
      config.Feedback.SensorToMechanismRatio = ElevatorConstants.GEARING_MOTOR_TO_ELEVATOR

      config.CurrentLimits.StatorCurrentLimitEnable = true
      config.CurrentLimits.SupplyCurrentLimitEnable = true
      config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.STATOR_LIM
      config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_LIM

      /** If we gonna have FOC in the future
       config.TorqueCurrent.PeakForwardTorqueCurrent = torqueCurrentLimit.`in`(Amps)
       config.TorqueCurrent.PeakReverseTorqueCurrent = -torqueCurrentLimit.`in`(Amps)
       **/

      config.Slot0.kP = ElevatorConstants.KP
      config.Slot0.kI = ElevatorConstants.KI
      config.Slot0.kD = ElevatorConstants.KD

      config.Slot0.kS = ElevatorConstants.KS
      config.Slot0.kV = ElevatorConstants.KV

      config.Slot1.kP = ElevatorConstants.KP
      config.Slot1.kI = ElevatorConstants.L4_KI
      config.Slot1.kD = ElevatorConstants.KD

      config.Slot1.kS = ElevatorConstants.KS
      config.Slot1.kV = ElevatorConstants.KV

      config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.CRUISE_VEL
      config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MAX_ACCEL

      val status1 = leadMotor.configurator.apply(config)
      if (!status1.isOK) println("Error applying configs to Elevator Lead Motor -> Error Code: $status1")

      val status2 = followerMotor.configurator.apply(config)
      if (!status2.isOK) println("Error applying configs to Elevator Follower Motor -> Error Code: $status2")

      BaseStatusSignal.setUpdateFrequencyForAll(
        ElevatorConstants.VALUE_UPDATE_RATE,
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

      leadMotor.optimizeBusUtilization()

      BaseStatusSignal.setUpdateFrequencyForAll(
        ElevatorConstants.VALUE_UPDATE_RATE,
        followerMotor.position,
        followerMotor.velocity,
        followerMotor.motorVoltage,
        followerMotor.supplyCurrent,
        followerMotor.statorCurrent,
        followerMotor.closedLoopReference,
        followerMotor.closedLoopReferenceSlope,
        followerMotor.closedLoopFeedForward,
        followerMotor.deviceTemp
      )

      followerMotor.optimizeBusUtilization()

      followerMotor.setControl(
        Follower(ElevatorConstants.LEAD_MOTOR_ID, ElevatorConstants.FOLLOWER_INVERTED_TO_MASTER)
      )

      return if (RobotBase.isReal()) Elevator(leadMotor) else ElevatorSim(leadMotor)
    }
  }
}
