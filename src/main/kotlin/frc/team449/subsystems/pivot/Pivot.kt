package frc.team449.subsystems.pivot

import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.subsystems.SuperstructureConstants
import frc.team449.subsystems.elevator.ElevatorConstants
import frc.team449.system.motor.createKraken
import java.util.function.Supplier
import kotlin.math.abs

class Pivot(
  private val motor: TalonFX
) : SubsystemBase() {

  val positionSupplier = Supplier { motor.position.valueAsDouble }
  val velocitySupplier = Supplier { motor.velocity.valueAsDouble }
  val targetSupplier = Supplier { request.Position }

  lateinit var pivotFeedForward: PivotFeedForward

  val pivotSim = LengthPivotSim(
    DCMotor.getKrakenX60(1),
    1 / PivotConstants.GEARING,
    PivotConstants.MOMENT_OF_INERTIA,
    PivotConstants.ARM_LENGTH,
    PivotConstants.MIN_ANGLE,
    PivotConstants.MAX_ANGLE,
    false,
    PivotConstants.MIN_ANGLE
  )

  val simPositionSupplier = Supplier { pivotSim.angleRads }

  private val request: MotionMagicVoltage = MotionMagicVoltage(
    SuperstructureConstants.STOW_POSITIONS.first
  )

  private fun setPosition(position: Double): Command {
    return this.runOnce {
      motor.setControl(
        request
          .withPosition(position)
          .withFeedForward(pivotFeedForward.calculateWithAngle(positionSupplier.get(), motor.closedLoopReferenceSlope.valueAsDouble, 0.0))
      )
    }
  }

  fun manualDown(): Command {
    return runOnce { motor.setVoltage(-3.0) }
  }

  fun manualUp(): Command {
    return runOnce { motor.setVoltage(3.0) }
  }

  fun stow(): Command {
    return setPosition(SuperstructureConstants.STOW_POSITIONS.first)
  }

  fun L1(): Command {
    return setPosition(SuperstructureConstants.L1_POSITIONS.first)
  }

  fun L2(): Command {
    return setPosition(SuperstructureConstants.L2_POSITIONS.first)
  }

  fun L3(): Command {
    return setPosition(SuperstructureConstants.L3_POSITIONS.first)
  }

  fun L4(): Command {
    return setPosition(SuperstructureConstants.L4_POSITIONS.first)
  }

  fun hold(): Command {
    return setPosition(targetSupplier.get())
  }

  fun stop(): Command {
    return this.run { motor.stopMotor() }
  }

  private fun atSetpoint(): Boolean {
    return (abs(positionSupplier.get() - request.Position) < ElevatorConstants.TOLERANCE)
  }

  override fun periodic() {}

  override fun simulationPeriodic() {
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
      val motor: TalonFX = createKraken(PivotConstants.MOTOR_ID, false)

      return Pivot(motor)
    }
  }
}
