package frc.team449.subsystems.superstructure

import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.Robot
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.elevator.Elevator
import frc.team449.subsystems.pivot.Pivot
import frc.team449.subsystems.wrist.Wrist

class SuperstructureManager(
  private val elevator: Elevator,
  private val pivot: Pivot,
  private val wrist: Wrist,
  private val drive: SwerveDrive
) {

  fun requestGoal(goal: SuperstructureGoal.SuperstructureState): Command {
    return InstantCommand({ SuperstructureGoal.applyDriveDynamics(drive, goal.driveDynamics) })
      .andThen(
        ConditionalCommand(
          // if extending
          pivot.setPosition(goal.pivot.`in`(Radians))
            .andThen(
              elevator.setPosition(goal.elevator.`in`(Meters))
                .alongWith(
                  wrist.setPosition(goal.wrist.`in`(Radians))
                )
            ),
          // if retracting
          elevator.setPosition(goal.elevator.`in`(Meters))
            .alongWith(wrist.setPosition(goal.wrist.`in`(Radians)))
            .andThen(pivot.setPosition((goal.pivot.`in`(Radians))))
        ) { goal.elevator.`in`(Meters) >= elevator.positionSupplier.get() }
      )
  }

  companion object {
    fun createSuperstructureManager(robot: Robot): SuperstructureManager {
      return SuperstructureManager(
        robot.elevator,
        robot.pivot,
        robot.wrist,
        robot.drive
      )
    }
  }
}
