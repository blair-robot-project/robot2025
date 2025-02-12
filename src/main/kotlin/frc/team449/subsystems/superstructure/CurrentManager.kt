package frc.team449.subsystems.superstructure

import com.ctre.phoenix6.configs.TalonFXConfiguration
import edu.wpi.first.wpilibj.PowerDistribution
import frc.team449.Robot
import frc.team449.subsystems.superstructure.elevator.Elevator
import frc.team449.subsystems.superstructure.pivot.Pivot
import frc.team449.subsystems.superstructure.wrist.Wrist
class CurrentManager(
  private val pdh: PowerDistribution
) {
    // define the channels for the motors
    private val pivotChannel = 0 //placeholders
    private val pivotChannel2 = 1 //placeholders

    private val elevatorChannel = 2 //placeholders
    private val elevatorChannel2 = 4 //placeholders

    private val wristChannel = 3 //placeholders

    //supply & stator limit for motor
    private val SUPPLY_LIM = 40.0
    private val STATOR_LIM = 80.0


    // get how much is being drawn
    private fun getMotorCurrentUsage(channel: Int): Double {
      return pdh.getCurrent(channel)
    }

  fun elevatorSUPPLY_LIM(): Double {
    val pivotCurrent = getMotorCurrentUsage(pivotChannel) + getMotorCurrentUsage(pivotChannel2)
    val wristCurrent = getMotorCurrentUsage(wristChannel)
    val elevatorCurrent = getMotorCurrentUsage(elevatorChannel) + getMotorCurrentUsage(elevatorChannel2)

    return if (pivotCurrent < 0 || wristCurrent < 0 || elevatorCurrent > 0.5) {
      SUPPLY_LIM * 1.5
    } else {
      SUPPLY_LIM
    }
  }


  fun pivotSUPPLY_LIM(): Double {
    val pivotCurrent = getMotorCurrentUsage(pivotChannel)
    val wristCurrent = getMotorCurrentUsage(wristChannel)
    val elevatorCurrent = getMotorCurrentUsage(elevatorChannel)

    return if (pivotCurrent > 0.5 || wristCurrent < 0.5 || elevatorCurrent < 0.5) {
      SUPPLY_LIM * 1.5
    } else {
      SUPPLY_LIM
    }
  }

  fun wristSUPPLY_LIM(
  ): Double {
    val pivotCurrent = getMotorCurrentUsage(pivotChannel)
    val wristCurrent = getMotorCurrentUsage(wristChannel)
    val elevatorCurrent = getMotorCurrentUsage(elevatorChannel)

    return if (pivotCurrent < 0.5 || wristCurrent > 0 || elevatorCurrent < 0.5) {
      SUPPLY_LIM * 1.5
    } else {
      SUPPLY_LIM
    }
  }

}