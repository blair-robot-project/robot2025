package frc.team449.subsystems.superstructure.elevator

import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.sim.TalonFXSimState
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import frc.team449.subsystems.RobotConstants
import kotlin.math.PI

class ElevatorSim(
  private val motor: TalonFX
) : Elevator(motor) {

  override val elevatorSim: TiltedElevatorSim = TiltedElevatorSim(
    DCMotor.getKrakenX60(2),
    1 / ElevatorConstants.GEARING_MOTOR_TO_PULLEY,
    ElevatorConstants.CARRIAGE_MASS,
    ElevatorConstants.PULLEY_RADIUS,
    ElevatorConstants.SIM_MIN_HEIGHT,
    ElevatorConstants.SIM_MAX_HEIGHT,
    simulateGravity = false,
    PI / 12
  )

  override fun periodic() {
    super.periodic()

    val motorSim: TalonFXSimState = motor.simState

    motorSim.setSupplyVoltage(12.0)
    val motorSimVoltage = motorSim.motorVoltage

    elevatorSim.setInputVoltage(MathUtil.clamp(motorSimVoltage, -12.0, 12.0))
//    println("${request.Position}  -  ${elevatorSim.positionMeters}")
    elevatorSim.update(RobotConstants.LOOP_TIME)

    motorSim.setRawRotorPosition(elevatorSim.positionMeters * ElevatorConstants.GEARING_MOTOR_TO_ELEVATOR)
    motorSim.setRotorVelocity(elevatorSim.velocityMetersPerSecond * ElevatorConstants.GEARING_MOTOR_TO_ELEVATOR)
  }
}
