package frc.team449.subsystems.drive.swerve

import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.spark.SparkMax
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState

interface SwerveModule {

  val location: Translation2d

  val desiredState: SwerveModuleState

  /** The module's [SwerveModuleState], containing speed and angle. */
  var state: SwerveModuleState

  /** The module's [SwerveModulePosition], containing distance and angle. */
  val position: SwerveModulePosition

  fun setVoltage(volts: Double) {}

  /** Set module speed to zero but keep module angle the same. */
  fun stop() {}

  fun update() {}

  val sprkDriv: SparkMax
  val krknDriv: TalonFX

  val turn: SparkMax
}
