package frc.team449.subsystems.superstructure

import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*

class AutoTestConstants {
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
    val WRIST_HARDSTOP_FRONT = Units.degreesToRadians(150.590515)

    const val WAIT_BETWEEN_PIVOT_TESTS = 0.25
    const val WAIT_BETWEEN_ELEVATOR_TESTS = 0.6
    const val WAIT_BETWEEN_WRIST_TESTS = 0.45

    const val WAIT_BETWEEN_EXTERNAL_TESTS = 0.25

    val PIVOT_SETPOINT_ONE = PIVOT_HARDSTOP_FRONT - Units.degreesToRadians(2.5)
    val PIVOT_SETPOINT_TWO = (PIVOT_HARDSTOP_FRONT - PIVOT_HARDSTOP_BACK) / 2
    val PIVOT_SETPOINT_THREE = PIVOT_HARDSTOP_BACK + Units.degreesToRadians(2.5)
    val PIVOT_SETPOINT_FOUR = SuperstructureGoal.L1.pivot.`in`(Radians)
    val PIVOT_SETPOINT_FIVE = SuperstructureGoal.L3.pivot.`in`(Radians)
    val PIVOT_SETPOINT_SIX = Units.degreesToRadians(85.0)

    val ELEVATOR_SETPOINT_ONE= SuperstructureGoal.L2.elevator.`in`(Meters)
    val ELEVATOR_SETPOINT_TWO = SuperstructureGoal.L4.elevator.`in`(Meters)
    val ELEVATOR_SETPOINT_THREE = SuperstructureGoal.L1.elevator.`in`(Meters)
    val ELEVATOR_SETPOINT_FOUR = SuperstructureGoal.L3.elevator.`in`(Meters)

    val WRIST_SETPOINT_ONE = SuperstructureGoal.L1.wrist.`in`(Radians)
    val WRIST_SETPOINT_TWO = SuperstructureGoal.L3.wrist.`in`(Radians)
    val WRIST_SETPOINT_THREE = SuperstructureGoal.L2.wrist.`in`(Radians)
    val WRIST_SETPOINT_FOUR = SuperstructureGoal.L4.wrist.`in`(Radians)
    val WRIST_SETPOINT_FIVE = SuperstructureGoal.STOW.wrist.`in`(Radians)

  }

}