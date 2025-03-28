package frc.team449.subsystems.superstructure
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.Robot
import frc.team449.subsystems.drive.swerve.SwerveModuleKraken
import kotlin.math.abs

class BuiltInTests (private val robot: Robot) {
  private val drive = robot.drive
  private val intake = robot.intake
  private val pivot = robot.pivot
  private val elevator = robot.elevator
  private val wrist = robot.wrist

  private var modulesAtSetpoint = false

  private fun waitUntilDriveAtTolerance(speeds: ChassisSpeeds): Command {
    return InstantCommand({
      robot.drive.set(speeds)
      modulesAtSetpoint = false
    }).andThen(InstantCommand({
      var atSetpoint = true
      robot.drive.modules.forEach() {
        val krakenModule = it as? SwerveModuleKraken //we use krakens
        val angleDistance = abs(krakenModule?.turnController?.setpoint!! - krakenModule.state.angle.radians)
        if(angleDistance >= BITConstants.DRIVE_ANGLE_TOLERANCE) {
          atSetpoint = false
        }
        println(angleDistance)
      }
      modulesAtSetpoint = atSetpoint
    }).until { modulesAtSetpoint })
  }

  fun testDrive(): Command {
    return Commands.sequence(
      InstantCommand({
        val emptyCommand = InstantCommand()
        emptyCommand.addRequirements(drive)
        drive.defaultCommand = emptyCommand
      }),
      waitUntilDriveAtTolerance(ChassisSpeeds(2.0, 0.0, 0.0)),
      waitUntilDriveAtTolerance(ChassisSpeeds(0.0, 2.0, 0.0)),
      waitUntilDriveAtTolerance(ChassisSpeeds(1.0, 1.0, 0.0)),
      waitUntilDriveAtTolerance(ChassisSpeeds(-1.0, -1.0, 0.0)),
      InstantCommand({drive.defaultCommand = robot.driveCommand})
    )
  }

  fun testIntake(): Command {
    return Commands.sequence(
      intake.intakeCoral(),
      WaitCommand(2.0),
      intake.outtakeCoral(),
      WaitCommand(1.0),
      intake.stop()
    )
  }
}