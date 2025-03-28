package frc.team449.subsystems.superstructure

import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*

class BITConstants {
  companion object {

    const val ELEVATOR_TOLERANCE = 0.025 //m
    val PIVOT_TOLERANCE = Units.degreesToRadians(1.0) //deg
    val WRIST_TOLERANCE = Units.degreesToRadians(2.0) //deg

    const val ELEVATOR_EXPECTED_TIME = 1.5 //sec
    const val ELEVATOR_TIMEOUT = 3.0 //sec
    const val PIVOT_EXPECTED_TIME = 1.2 //sec
    const val PIVOT_TIMEOUT = 2.0 //sec
    const val WRIST_EXPECTED_TIME = 1.2 //sec
    const val WRIST_TIMEOUT = 2.0 //sec

    val PIVOT_HARDSTOP_BACK = Units.degreesToRadians(-4.790334) //deg
    val PIVOT_HARDSTOP_FRONT = Units.degreesToRadians(100.0) //deg
    val WRIST_HARDSTOP_BACK = Units.degreesToRadians(4.790334)
    val WRIST_HARDSTOP_FRONT = Units.degreesToRadians(220.928943)

    const val PIVOT_WAIT = 0.25 //s
    const val ELEVATOR_WAIT = 0.6 //s
    const val WRIST_WAIT = 0.45 //s
    const val EXTERNAL_WAIT = 0.75 //s

    const val PIVOT_FIRST_ROM_DIVIDE = 2.0
    const val ELEVATOR_FIRST_ROM_DIVIDE = 2.5
    const val WRIST_FIRST_ROM_DIVIDE = 2.5
    const val PIVOT_SECOND_ROM_DIVIDE = 1.5
    const val ELEVATOR_SECOND_ROM_DIVIDE = 1.65
    const val WRIST_SECOND_ROM_DIVIDE = 1.65

    const val INPUT_TIMEOUT = 7.5 //s

    val PIVOT_SETPOINT_ONE = PIVOT_HARDSTOP_FRONT - Units.degreesToRadians(2.5)
    val PIVOT_SETPOINT_TWO = (PIVOT_HARDSTOP_FRONT - PIVOT_HARDSTOP_BACK) / 2
    val PIVOT_SETPOINT_THREE = PIVOT_HARDSTOP_BACK + Units.degreesToRadians(2.5)
    val PIVOT_SETPOINT_FOUR = SuperstructureGoal.L1.pivot.`in`(Radians)
    val PIVOT_SETPOINT_FIVE = SuperstructureGoal.L3.pivot.`in`(Radians)
    val PIVOT_SETPOINT_SIX = Units.degreesToRadians(75.0)

    val ELEVATOR_SETPOINT_ONE = SuperstructureGoal.L2.elevator.`in`(Meters)
    val ELEVATOR_SETPOINT_TWO = SuperstructureGoal.L4.elevator.`in`(Meters)
    val ELEVATOR_SETPOINT_THREE = SuperstructureGoal.L1.elevator.`in`(Meters)
    val ELEVATOR_SETPOINT_FOUR = SuperstructureGoal.L3.elevator.`in`(Meters)

    val WRIST_SETPOINT_ONE = SuperstructureGoal.L1.wrist.`in`(Radians)
    val WRIST_SETPOINT_TWO = SuperstructureGoal.L3.wrist.`in`(Radians)
    val WRIST_SETPOINT_THREE = SuperstructureGoal.L2.wrist.`in`(Radians)
    val WRIST_SETPOINT_FOUR = SuperstructureGoal.L4.wrist.`in`(Radians)
    val WRIST_SETPOINT_FIVE = SuperstructureGoal.STOW.wrist.`in`(Radians)

    const val HIGH_PIVOT_VOLTAGE = 15.0
    const val HIGH_ELEVATOR_VOLTAGE = 17.5
    const val HIGH_WRIST_VOLTAGE = 12.5

    const val PIVOT_SLOW_VOLTAGE = 4.0
    const val PIVOT_MEDIUM_VOLTAGE = 4.0
    const val PIVOT_HIGH_VOLTAGE = 4.0


    val DRIVE_ANGLE_TOLERANCE = Units.degreesToRadians(5.0)
  }

}