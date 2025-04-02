package frc.team449.subsystems.voltage

import edu.wpi.first.units.Units.Amps
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.subsystems.drive.swerve.SwerveConstants
import frc.team449.subsystems.superstructure.elevator.ElevatorConstants
import frc.team449.subsystems.superstructure.pivot.PivotConstants
import frc.team449.subsystems.superstructure.wrist.WristConstants

class VoltageDistribution(
  val pdh: PowerDistribution
): SubsystemBase() {

  var pivotCurrent: Double
  var eleCurrent: Double
  var wristCurrent: Double
  var driveCurrent: Double

  var pivotSupply = 0.0
  var eleSupply = 0.0
  var wristSupply = 0.0
  var driveSupply = 0.0

  var totalSupply = PivotConstants.SUPPLY_LIM + ElevatorConstants.SUPPLY_LIM + WristConstants.SUPPLY_LIM + SwerveConstants.DRIVE_SUPPLY_LIMIT.`in`(Amps)

  init {
    pivotCurrent = pdh.getCurrent(VoltageDistributionConstants.pivotChannel) + pdh.getCurrent(VoltageDistributionConstants.pivotChannel2)
    eleCurrent = pdh.getCurrent(VoltageDistributionConstants.elevatorChannel) + pdh.getCurrent(VoltageDistributionConstants.elevatorChannel2)
    wristCurrent = pdh.getCurrent(VoltageDistributionConstants.wristChannel)
    driveCurrent = pdh.getCurrent(VoltageDistributionConstants.driveChannel)
  }

  fun updateSupplies() {
    pivotCurrent = pdh.getCurrent(VoltageDistributionConstants.pivotChannel) + pdh.getCurrent(VoltageDistributionConstants.pivotChannel2)
    eleCurrent = pdh.getCurrent(VoltageDistributionConstants.elevatorChannel) + pdh.getCurrent(VoltageDistributionConstants.elevatorChannel2)
    wristCurrent = pdh.getCurrent(VoltageDistributionConstants.wristChannel)
    driveCurrent = pdh.getCurrent(VoltageDistributionConstants.driveChannel)
  }

  fun calculateSupplies(): Array<Double> {
    /*
    supply, stator:
    pivot: 40, 80
    elevator: 50, 90
    wrist: 40, 80
    drive: 52.5, 105
     */
    pivotSupply = pivotCurrent/pdh.totalCurrent * totalSupply
    eleSupply = eleCurrent/pdh.totalCurrent * totalSupply
    wristSupply = wristCurrent/pdh.totalCurrent * totalSupply
    driveSupply = driveCurrent/pdh.totalCurrent * totalSupply

    return arrayOf(pivotSupply, eleSupply, wristSupply)
  }

  override fun periodic() {
    updateSupplies()
    println("calculated supplies from my thing: ${calculateSupplies().contentToString()}")

  }

}