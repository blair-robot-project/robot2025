package frc.team449.subsystems.superstructure

import com.ctre.phoenix6.configs.TalonFXConfiguration
import edu.wpi.first.wpilibj.PowerDistribution
import frc.team449.Robot
import frc.team449.subsystems.superstructure.elevator.Elevator
import frc.team449.subsystems.superstructure.elevator.ElevatorConstants
import frc.team449.subsystems.superstructure.pivot.Pivot
import frc.team449.subsystems.superstructure.wrist.Wrist


class powerDistribution(
  val robot: Robot,
  private val pdh: PowerDistribution,
  private val pivotMotor: Pivot,
  private val elevatorMotor: Elevator,
  private val wristMotor: Wrist
) {
    // define the channels for the motors
    private val pivotChannel = 0
    private val elevatorChannel = 1
    private val wristChannel = 2

    //stator limit for motor
    private val supplyCurrentLimit = 40.0
    private val statorCurrentLimit = 80.0


    // get how much is being drawn
    private fun getMotorCurrentUsage(channel: Int): Double {
      return pdh.getCurrent(channel)
    }

    fun adjustPowerDistribution() {
        val pivotCurrent = getMotorCurrentUsage(pivotChannel)
        val elevatorCurrent = getMotorCurrentUsage(elevatorChannel)
        val wristCurrent = getMotorCurrentUsage(wristChannel)


        // Total current being drawn
        val totalCurrent = pivotCurrent + elevatorCurrent + wristCurrent

        val idealTotalCurrent = supplyCurrentLimit * 6 // fix this 6 motors but only 3 mechanisms

        val config = TalonFXConfiguration()


fun elevatorset() {
  if (pivotCurrent < 0 || wristCurrent < 0 || elevatorCurrent > 0.5) {
    config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_LIM * 1.5
  }
}

      if (wristCurrent > 0 || elevatorCurrent > 0) {
        wristMotor.config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_LIM * 1.5
      }

      if (pivotCurrent > 0 || wristCurrent > 0) {
        pivotMotor.config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.SUPPLY_LIM * 1.5
      }


        }

    fun runPowerManagement() {
        adjustPowerDistribution()
    }
}

