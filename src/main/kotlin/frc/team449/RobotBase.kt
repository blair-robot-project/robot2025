package frc.team449

import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.vision.PoseSubsystem

abstract class RobotBase {

    val field = Field2d()

    abstract val powerDistribution: PowerDistribution

    abstract val drive: SwerveDrive

    abstract val poseSubsystem: PoseSubsystem

    abstract val driveCommand: Command
}
