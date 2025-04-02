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

    val PIVOT_HARDSTOP_BACK = Units.degreesToRadians(-4.790334)
    val PIVOT_HARDSTOP_FRONT = Units.degreesToRadians(100.0)
    val WRIST_HARDSTOP_BACK = Units.degreesToRadians(4.790334)
    val WRIST_HARDSTOP_FRONT = Units.degreesToRadians(220.928943)
    val ELEVATOR_HARDSTOP_TOP = SuperstructureGoal.L4.elevator.`in`(Meters)
    val ELEVATOR_HARDSTOP_BOTTOM = Inches.of(-0.35).`in`(Meters)

    const val PIVOT_WAIT = 0.25 //s
    const val ELEVATOR_WAIT = 0.6 //s
    const val WRIST_WAIT = 0.45 //s
    const val DRIVE_WAIT = 0.5 //s
    const val EXTERNAL_WAIT = 0.75 //s

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

    const val PIVOT_SLOW_VOLTAGE = 6.0
    const val PIVOT_MEDIUM_VOLTAGE = 8.0
    const val PIVOT_FAST_VOLTAGE = 10.0

    const val ELEVATOR_SLOW_VOLTAGE = 10.0
    const val ELEVATOR_MEDIUM_VOLTAGE = 14.0
    const val ELEVATOR_FAST_VOLTAGE = 18.0

    const val WRIST_SLOW_VOLTAGE = 6.0
    const val WRIST_MEDIUM_VOLTAGE = 8.0
    const val WRIST_FAST_VOLTAGE = 10.0

    val DRIVE_ANGLE_TOLERANCE = Units.degreesToRadians(5.0)
  }

}