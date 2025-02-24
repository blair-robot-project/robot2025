package frc.team449.auto.routines

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.Robot
import frc.team449.auto.AutoUtil
import frc.team449.auto.choreo.ChoreoRoutine
import frc.team449.auto.choreo.ChoreoRoutineStructure
import frc.team449.auto.choreo.ChoreoTrajectory
import frc.team449.commands.driveAlign.SimpleReefAlign
import frc.team449.subsystems.FieldConstants
import frc.team449.subsystems.superstructure.SuperstructureGoal
import java.util.Optional

class ThreeL4(
  robot: Robot,
  isRedAlliance: Boolean,
  isRedStage: Boolean
) : ChoreoRoutineStructure {

  override val routine =
    ChoreoRoutine(
      drive = robot.drive,
      poseSubsystem = robot.poseSubsystem,
      parallelEventMap = hashMapOf(
        0 to premoveL4(robot),
        1 to premoveSubstation(robot),
        2 to premoveL4(robot),
        3 to premoveSubstation(robot),
        4 to premoveL4(robot),
      ),
      stopEventMap = hashMapOf(
        1 to scoreL4(robot, if (isRedAlliance && isRedStage || !isRedAlliance && !isRedStage) FieldConstants.ReefSide.RIGHT else FieldConstants.ReefSide.LEFT),
        2 to intakeSubstation(robot),
        3 to scoreL4(robot, if (isRedAlliance && isRedStage || !isRedAlliance && !isRedStage) FieldConstants.ReefSide.LEFT else FieldConstants.ReefSide.RIGHT),
        4 to intakeSubstation(robot),
        5 to scoreL4(robot, if (isRedAlliance && isRedStage || !isRedAlliance && !isRedStage) FieldConstants.ReefSide.RIGHT else FieldConstants.ReefSide.LEFT)
          .andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)),
      ),
      debug = false,
      timeout = 0.0,
      stopDriveAfterTraj = false
    )

  private fun premoveL4(robot: Robot): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.L4_PREMOVE)
  }

  private fun premoveSubstation(robot: Robot): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
      .alongWith(robot.intake.intakeCoral())
  }

  private fun scoreL4(robot: Robot, reefSide: FieldConstants.ReefSide): Command {
    return robot.superstructureManager.requestGoal(SuperstructureGoal.L4)
      .alongWith(SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(reefSide), translationSpeedLim = 2.0))
      .andThen(robot.intake.outtakeCoral())
      .andThen(
        WaitUntilCommand { !robot.intake.coralDetected() }
          .onlyIf { RobotBase.isReal() }
      )
      .andThen(WaitCommand(0.15))
      .andThen(robot.intake.stop())
//    return SimpleReefAlign(robot.drive, robot.poseSubsystem, leftOrRight = Optional.of(reefSide), translationSpeedLim = 2.0)
//      .andThen(WaitCommand(1.0))
  }

  private fun intakeSubstation(robot: Robot): Command {
    return InstantCommand(robot.drive::stop)
      .andThen(robot.intake.intakeCoral())
      .andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE))
      .andThen(
        WaitUntilCommand { robot.intake.coralDetected() }
          .onlyIf { RobotBase.isReal() }
      )
      .andThen(WaitCommand(0.15))
      .andThen(robot.intake.holdCoral())
//    return InstantCommand(robot.drive::stop).andThen(
//      WaitCommand(1.5)
//    )
  }

  override val trajectory: MutableList<ChoreoTrajectory> =
    if (isRedAlliance) {
      if (isRedStage) {
        AutoUtil.transformForRedAlliance(
          ChoreoTrajectory.createTrajectory(arrayListOf("1", "2", "3", "4", "5"), "ThreeL4")
        )
      } else {
        AutoUtil.transformForBlueStage(
          AutoUtil.transformForRedAlliance(
            ChoreoTrajectory.createTrajectory(arrayListOf("1", "2", "3", "4", "5"), "ThreeL4")
          )
        )
      }
    } else {
      if (isRedStage) {
        ChoreoTrajectory.createTrajectory(arrayListOf("1", "2", "3", "4", "5"), "ThreeL4")
      } else {
        AutoUtil.transformForBlueStage(
          ChoreoTrajectory.createTrajectory(arrayListOf("1", "2", "3", "4", "5"), "ThreeL4")
        )
      }
    }
}
