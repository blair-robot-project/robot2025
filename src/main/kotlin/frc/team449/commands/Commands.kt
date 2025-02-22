package frc.team449.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.Robot
import frc.team449.commands.driveAlign.SimpleReefAlign
import frc.team449.subsystems.FieldConstants
import frc.team449.subsystems.superstructure.SuperstructureGoal
import java.util.Optional

object Commands {
  // Make robot specific commands such as stowing, intaking, scoring, etc.

  fun ScoreL4(robot: Robot, leftOrRight: FieldConstants.ReefSide): Command {
    return Commands.sequence(
      Commands.parallel(
        SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(leftOrRight)),
        robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
      ),
      robot.drive.driveStop(),
      robot.intake.outtakeCoral(),
      WaitUntilCommand(robot.intake::coralNotDetected),
    )
  }

  fun Intake(robot: Robot): Command {
    return Commands.sequence(
      robot.intake.intakeCoral(),
      WaitUntilCommand(robot.intake::coralDetected),
      robot.intake.holdCoral()
    )
  }
}
