package frc.team449.subsystems.superstructure.intake

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import kotlin.math.PI

object IntakeConstants {
  const val TOP_MOTOR_ID = 47
  const val RIGHT_MOTOR_ID = 46
  const val LEFT_MOTOR_ID = 48

  const val RIGHT_CORAL_SENSOR_CAN_ID = 20
  const val MIDDLE_CORAL_SENSOR_CAN_ID = 21
  const val LEFT_CORAL_SENSOR_CAN_ID = 22
  const val BACK_CORAL_SENSOR_CAN_ID = 23

  const val KP = 3.8
  const val KI = 0.0
  const val KD = 0.0254
  const val KS = 0.4

  const val WHEEL_TOLERANCE = 0.035

  val TOP_INVERTED = InvertedValue.Clockwise_Positive
  val RIGHT_INVERTED = InvertedValue.CounterClockwise_Positive
  val LEFT_INVERTED = InvertedValue.Clockwise_Positive
  val BRAKE_MODE = NeutralModeValue.Brake

  const val SUPPLY_LIM = 40.0
  const val STATOR_LIM = 80.0

  // configs
  val config = TalonFXConfiguration()
  val TOP_MOTOR_INTAKING_CONFIG: CurrentLimitsConfigs =
    CurrentLimitsConfigs()
      .withSupplyCurrentLimit(40.0)
      .withStatorCurrentLimit(80.0)
  val TOP_MOTOR_HOLDING_CONFIG: CurrentLimitsConfigs =
    CurrentLimitsConfigs()
      .withSupplyCurrentLimit(30.0)
      .withStatorCurrentLimit(100.0)
  val TOP_MOTOR_HOLDING_CONFIG_PROC: CurrentLimitsConfigs =
    CurrentLimitsConfigs()
      .withSupplyCurrentLimit(25.0)
      .withStatorCurrentLimit(75.0)

  // debouncers
  const val ALGAE_DEBOUNCER_WAIT = 0.4
  const val L1_DEBOUNCER_WAIT = 0.15
  const val VERTICAL_DEBOUNCER_WAIT = 0.0
  const val PIECE_RESET_DEBOUNCER_WAIT = 0.2
  const val PIECE_DETECT_DEBOUNCER_WAIT = 1.1

  // moving constants
  const val PIVOT_MOVEMENT = 3.5
  const val OPP_MOVEMENT = 3.5
  const val INTAKEN_TO_CENTERED = 1.0

  // voltage for different scenarios and motors
  const val TOP_CORAL_INWARDS_VOLTAGE = 10.0
  const val TOP_CORAL_OUTTAKE_VOLTAGE = -7.0
  const val TOP_L1_HOLD = 1.690
  const val TOP_L1_OUTTAKE = -2.1678
  const val TOP_MOTOR_UNVERTICALING_SLOWDOWN = 10.0
  const val TOP_MOTOR_HORIZONTAL_SLOWDOWN = 3.0

  const val SIDES_CORAL_INWARDS_VOLTAGE = 6.0
  const val SIDES_CORAL_OUTTAKE_VOLTAGE = -6.0
  const val SIDES_RUN_TO_SIDE_VOLTAGE = 8.0
  const val SIDES_SLOWDOWN_CONSTANT = 1.2

  const val ALGAE_INTAKE_VOLTAGE = -11.0
  const val ALGAE_HOLD_VOLTAGE = -7.0
  const val ALGAE_OUTTAKE_VOLTAGE = 11.0
  const val ALGAE_STALL_VOLTAGE_THRESHOLD = 60.0

  const val SLIDE_CORAL_TO_OPP = -PI // radians
  const val SLIDE_CORAL_TO_PIVOT = PI // radians

  const val HOLD_ANGLE_CHANGE = 0.5
  const val HOLDING_FINISH_VELOCITY = 0.125

  const val WAIT_BEFORE_ALGAE_IN = 0.8
  const val WAIT_BEFORE_ALGAE_OUT = 0.15

  // Minimum distance in mm on the LaserCAN sensors to count as a detection
  const val CORAL_DETECTION_THRESHOLD = 50

  init {
    config.MotorOutput.NeutralMode = BRAKE_MODE

    config.CurrentLimits.StatorCurrentLimitEnable = true
    config.CurrentLimits.SupplyCurrentLimitEnable = true
    config.CurrentLimits.SupplyCurrentLimit = SUPPLY_LIM
    config.CurrentLimits.StatorCurrentLimit = STATOR_LIM
  }
}
