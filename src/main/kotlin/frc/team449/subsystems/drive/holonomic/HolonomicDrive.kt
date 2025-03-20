package frc.team449.subsystems.drive.holonomic

import frc.team449.subsystems.drive.DriveSubsystem

interface HolonomicDrive : DriveSubsystem {
  /** The max speed that the drivetrain can translate at, in meters per second. */
  var maxLinearSpeed: Double

  /** The max speed that the drivetrain can turn in place at, in radians per second. */
  var maxRotSpeed: Double
}
