package frc.team449.subsystems.voltage

import com.ctre.phoenix6.hardware.TalonFX
import dev.doglog.DogLog
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Robot
import frc.team449.subsystems.drive.swerve.SwerveConstants
import frc.team449.subsystems.superstructure.elevator.ElevatorConstants
import frc.team449.subsystems.superstructure.pivot.PivotConstants
import frc.team449.subsystems.superstructure.wrist.WristConstants

class VoltageDistribution(
  val pdh: PowerDistribution,
  val robot: Robot
): SubsystemBase() {

  lateinit var pivotMotor1: TalonFX
  lateinit var pivotMotor2: TalonFX
  lateinit var eleMotor1: TalonFX
  lateinit var eleMotor2: TalonFX
  lateinit var wristMotor: TalonFX

  var pivotCurrent = 0.0
  var eleCurrent = 0.0
  var wristCurrent = 0.0
  var driveCurrent = 0.0
  var driveeCurrent = 0.0

  var pivotSupply = 0.0
  var eleSupply = 0.0
  var wristSupply = 0.0
  var driveSupply = 0.0

  var totalSupply = PivotConstants.SUPPLY_LIM + ElevatorConstants.SUPPLY_LIM + WristConstants.SUPPLY_LIM + SwerveConstants.DRIVE_SUPPLY_LIMIT.`in`(Amps)

  init {
    //pivotCurrent = pdh.getCurrent(VoltageDistributionConstants.pivotChannel) + pdh.getCurrent(VoltageDistributionConstants.pivotChannel2)
    if (robot.pivot != null && robot.elevator != null && robot.wrist != null && robot.drive != null) {
      //pivotCurrent = robot.pivot.motor.getSupplyCurrent(true).valueAsDouble
      pivotCurrent =
        pivotMotor1.getSupplyCurrent(true).valueAsDouble +
                pivotMotor2.getSupplyCurrent(true).valueAsDouble
      //pivotCurrent = robot.pivot.returnMotor().getSupplyCurrent(true).valueAsDouble
      //eleCurrent = pdh.getCurrent(VoltageDistributionConstants.elevatorChannel) + pdh.getCurrent(VoltageDistributionConstants.elevatorChannel2)
//      eleCurrent = robot.elevator.motor.getSupplyCurrent(true).valueAsDouble
      eleCurrent =
              eleMotor1.getSupplyCurrent(true).valueAsDouble +
                      eleMotor2.getSupplyCurrent(true).valueAsDouble
      //wristCurrent = pdh.getCurrent(VoltageDistributionConstants.wristChannel)
      //wristCurrent = robot.wrist.motor.getSupplyCurrent(true).valueAsDouble
      wristCurrent = wristMotor.getSupplyCurrent(true).valueAsDouble
//      driveCurrent =
//        pdh.getCurrent(VoltageDistributionConstants.driveChannel1) + pdh.getCurrent(VoltageDistributionConstants.driveChannel2) + pdh.getCurrent(VoltageDistributionConstants.driveChannel3) + pdh.getCurrent(
//          VoltageDistributionConstants.driveChannel4
//        )
      robot.drive.modules.forEach {
        driveeCurrent += it.krknDriv.supplyCurrent.valueAsDouble
        driveeCurrent += it.turn.busVoltage
      }
      driveCurrent = driveeCurrent
      driveeCurrent = 0.0
    }
  }

  fun updateSupplies() {
    //pivotCurrent = pdh.getCurrent(VoltageDistributionConstants.pivotChannel) + pdh.getCurrent(VoltageDistributionConstants.pivotChannel2)
    if (robot.pivot != null && robot.elevator != null && robot.wrist != null && robot.drive != null) {
      //pivotCurrent = robot.pivot.motor.getSupplyCurrent(true).valueAsDouble
      pivotCurrent =
        pivotMotor1.getSupplyCurrent(true).valueAsDouble +
                pivotMotor2.getSupplyCurrent(true).valueAsDouble
      //pivotCurrent = robot.pivot.returnMotor().getSupplyCurrent(true).valueAsDouble
      //eleCurrent = pdh.getCurrent(VoltageDistributionConstants.elevatorChannel) + pdh.getCurrent(VoltageDistributionConstants.elevatorChannel2)
//      eleCurrent = robot.elevator.motor.getSupplyCurrent(true).valueAsDouble
      eleCurrent =
        eleMotor1.getSupplyCurrent(true).valueAsDouble +
                eleMotor2.getSupplyCurrent(true).valueAsDouble
      //wristCurrent = pdh.getCurrent(VoltageDistributionConstants.wristChannel)
      //wristCurrent = robot.wrist.motor.getSupplyCurrent(true).valueAsDouble
      wristCurrent = wristMotor.getSupplyCurrent(true).valueAsDouble
//      driveCurrent =
//        pdh.getCurrent(VoltageDistributionConstants.driveChannel1) + pdh.getCurrent(VoltageDistributionConstants.driveChannel2) + pdh.getCurrent(VoltageDistributionConstants.driveChannel3) + pdh.getCurrent(
//          VoltageDistributionConstants.driveChannel4
//        )
      robot.drive.modules.forEach {
        driveeCurrent += it.krknDriv.supplyCurrent.valueAsDouble
        driveeCurrent += it.turn.busVoltage
      }
      driveCurrent = driveeCurrent
      driveeCurrent = 0.0
    }
  }

  fun calculateSupplies(): Array<Double> {
    /*
    supply, stator:
    pivot: 40, 80
    elevator: 50, 90
    wrist: 40, 80
    drive: 52.5, 105
     */
//    pivotSupply = pivotCurrent/pdh.totalCurrent * totalSupply
//    eleSupply = eleCurrent/pdh.totalCurrent * totalSupply
//    wristSupply = wristCurrent/pdh.totalCurrent * totalSupply
//    driveSupply = driveCurrent/pdh.totalCurrent * totalSupply

    pivotSupply = pivotCurrent/(pivotSupply + eleSupply + wristSupply + driveSupply) * totalSupply
    eleSupply = eleCurrent/(pivotSupply + eleSupply + wristSupply + driveSupply) * totalSupply
    wristSupply = wristCurrent/(pivotSupply + eleSupply + wristSupply + driveSupply) * totalSupply
    driveSupply = driveCurrent/(pivotSupply + eleSupply + wristSupply + driveSupply) * totalSupply

    return arrayOf(pivotSupply, eleSupply, wristSupply, driveSupply)
  }

  override fun periodic() {
    println("elevator id: ${robot.elevator.motor.deviceID}")
    println("elevator acceleration??: ${eleMotor1.acceleration}")
    DogLog.log("/VoltageDistribution/ele acceleration", eleMotor1.acceleration.valueAsDouble)
    updateSupplies()
    calculateSupplies()
    println("currents -- pivot: $pivotCurrent, elevator: $eleCurrent, wrist: $wristCurrent, drive: $driveCurrent")
    DogLog.log("/VoltageDistribution/ele current", eleCurrent)
  }

}