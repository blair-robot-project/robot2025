package frc.team449.subsystems.elevator

import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.SuperstructureConstants
import frc.team449.subsystems.wrist.WristConstants
import frc.team449.system.motor.createKraken
import java.util.function.Supplier
import kotlin.math.PI
import kotlin.math.abs

open class Elevator(private val motor: TalonFX) : SubsystemBase() {

    open val positionSupplier = Supplier { motor.position.valueAsDouble }
    open val velocitySupplier = Supplier { motor.velocity.valueAsDouble }
    open val targetSupplier = Supplier { request.Position }

    val elevatorSim: TiltedElevatorSim =
        TiltedElevatorSim(
            DCMotor.getKrakenX60(1),
            1 / ElevatorConstants.GEARING,
            ElevatorConstants.CARRIAGE_MASS,
            ElevatorConstants.PULLEY_RADIUS,
            ElevatorConstants.MIN_HEIGHT,
            ElevatorConstants.MAX_HEIGHT,
            simulateGravity = false,
            PI / 12,
        )

    val simPositionSupplier = Supplier { elevatorSim.positionMeters }

    val mech: Mechanism2d = Mechanism2d(3.0, 3.0, Color8Bit(0, 0, 0))
    val rootElevator: MechanismRoot2d = mech.getRoot("elevatorRoot", 0.25, 0.25)
    val elevatorLigament: MechanismLigament2d =
        rootElevator.append(
            MechanismLigament2d(
                "elevatorLigament",
                ElevatorConstants.MIN_HEIGHT,
                ElevatorConstants.ANGLE,
                ElevatorConstants.WIDTH,
                ElevatorConstants.COLOR,
            )
        )

    val desiredElevatorLigament: MechanismLigament2d =
        rootElevator.append(
            MechanismLigament2d(
                "elevatorDesiredLigament",
                ElevatorConstants.MIN_HEIGHT,
                ElevatorConstants.ANGLE,
                ElevatorConstants.DESIRED_WIDTH,
                ElevatorConstants.DESIRED_COLOR,
            )
        )

    val wristLigament: MechanismLigament2d =
        elevatorLigament.append(
            MechanismLigament2d(
                "wristLigament",
                0.2,
                WristConstants.ANGLE,
                WristConstants.WIDTH,
                WristConstants.COLOR,
            )
        )

    private val request: MotionMagicVoltage =
        MotionMagicVoltage(SuperstructureConstants.STOW_POSITIONS.second)

    private fun setPosition(position: Double): Command {
        return this.runOnce {
            motor.setControl(request.withPosition(position).withFeedForward(0.0))
        }
    }

    fun manualDown(): Command {
        return runOnce { motor.setVoltage(-3.0) }
    }

    fun manualUp(): Command {
        return runOnce { motor.setVoltage(3.0) }
    }

    fun stow(): Command {
        return setPosition(SuperstructureConstants.STOW_POSITIONS.second)
    }

    fun L1(): Command {
        return setPosition(SuperstructureConstants.L1_POSITIONS.second)
    }

    fun L2(): Command {
        return setPosition(SuperstructureConstants.L2_POSITIONS.second)
    }

    fun L3(): Command {
        return setPosition(SuperstructureConstants.L3_POSITIONS.second)
    }

    fun L4(): Command {
        return setPosition(SuperstructureConstants.L4_POSITIONS.second)
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
        elevatorSim.setInputVoltage(MathUtil.clamp(motor.motorVoltage.valueAsDouble, -12.0, 12.0))
        elevatorSim.update(RobotConstants.LOOP_TIME)
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
            val elevatorMotor: TalonFX = createKraken(ElevatorConstants.MOTOR_ID, false)
            return Elevator(elevatorMotor)
        }
    }
}
