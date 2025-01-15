package frc.team449.subsystems.elevator

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.sim.TalonFXSimState
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.*
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.superstructure.SuperstructureGoal
import frc.team449.subsystems.wrist.WristConstants
import frc.team449.system.motor.createKraken
import java.util.function.Supplier
import kotlin.math.PI
import kotlin.math.abs

open class Elevator(
  private val motor: TalonFX
) : SubsystemBase() {

  open val positionSupplier = Supplier { motor.position.valueAsDouble }
  open val velocitySupplier = Supplier { motor.velocity.valueAsDouble }
  open val targetSupplier = Supplier { request.Position }

  lateinit var elevatorFeedForward: ElevatorFeedForward

  val elevatorSim: TiltedElevatorSim = TiltedElevatorSim(
    DCMotor.getKrakenX60(1),
    1 / ElevatorConstants.GEARING,
    ElevatorConstants.CARRIAGE_MASS,
    ElevatorConstants.PULLEY_RADIUS,
    ElevatorConstants.MIN_HEIGHT,
    ElevatorConstants.MAX_HEIGHT,
    simulateGravity = false,
    PI / 12
  )

  val mech: Mechanism2d = Mechanism2d(3.0, 3.0, Color8Bit(0, 0, 0))
  private val rootElevator: MechanismRoot2d = mech.getRoot("elevatorRoot", 0.25, 0.25)
  val elevatorLigament: MechanismLigament2d = rootElevator.append(
    MechanismLigament2d(
      "elevatorLigament",
      ElevatorConstants.MIN_HEIGHT,
      ElevatorConstants.ANGLE,
      ElevatorConstants.WIDTH,
      ElevatorConstants.COLOR
    )
  )

  val desiredElevatorLigament: MechanismLigament2d = rootElevator.append(
    MechanismLigament2d(
      "elevatorDesiredLigament",
      ElevatorConstants.MIN_HEIGHT,
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
  )

  fun setPosition(position: Double): Command {
    return this.run {
      motor.setControl(
        request
          .withPosition(position)
          .withFeedForward(
            elevatorFeedForward.calculate(motor.closedLoopReferenceSlope.valueAsDouble)
          )
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
    return (abs(positionSupplier.get() - request.Position) < ElevatorConstants.TOLERANCE)
  }

  override fun periodic() {}

  override fun simulationPeriodic() {
    val motorSim: TalonFXSimState = motor.simState

    motorSim.setSupplyVoltage(12.0)
    val motorSimVoltage = motorSim.motorVoltage

    elevatorSim.setInputVoltage(MathUtil.clamp(motorSimVoltage, -12.0, 12.0))
//    println("${request.Position}  -  ${elevatorSim.positionMeters}")
    elevatorSim.update(RobotConstants.LOOP_TIME)

    motorSim.setRawRotorPosition(elevatorSim.positionMeters / (ElevatorConstants.UPR * ElevatorConstants.GEARING))
    motorSim.setRotorVelocity(elevatorSim.velocityMetersPerSecond / (ElevatorConstants.UPR * ElevatorConstants.GEARING))
  }

  override fun initSendable(builder: SendableBuilder) {
    builder.publishConstString("1.0", "Elevator Info")
    builder.addDoubleProperty("1.1 Voltage", { motor.motorVoltage.valueAsDouble }, null)
    builder.addDoubleProperty("1.2 Position", { positionSupplier.get() }, null)
    builder.addDoubleProperty("1.3 Velocity", { velocitySupplier.get() }, null)
    builder.addDoubleProperty("1.4 Desired Position", { request.Position }, null)
    builder.addBooleanProperty("1.5 At Tolerance", { atSetpoint() }, null)
    // builder.addStringProperty("1.7 Command", {this.currentCommand.name}, null)
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
      config.Feedback.SensorToMechanismRatio = 1 / (ElevatorConstants.GEARING * ElevatorConstants.UPR)

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

      config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.CRUISE_VEL
      config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MAX_ACCEL

      val status1 = leadMotor.configurator.apply(config)
      if (!status1.isOK) println("Could not apply configs to lead elevator motor, error code: $status1")

      val status2 = followerMotor.configurator.apply(config)
      if (!status1.isOK) println("Could not apply configs to follower elevator motor, error code: $status2")

      BaseStatusSignal.setUpdateFrequencyForAll(
        ElevatorConstants.VALUE_UPDATE_RATE,
        leadMotor.position,
        leadMotor.velocity,
        leadMotor.motorVoltage,
        leadMotor.supplyCurrent,
        leadMotor.statorCurrent,
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
        followerMotor.deviceTemp
      )

      followerMotor.optimizeBusUtilization()

      followerMotor.setControl(
        Follower(ElevatorConstants.LEAD_MOTOR_ID, ElevatorConstants.FOLLOWER_INVERTED_TO_MASTER)
      )

      return Elevator(leadMotor)
    }
  }
}
