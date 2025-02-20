package frc.team449.subsystems.superstructure

import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.Robot
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.superstructure.elevator.Elevator
import frc.team449.subsystems.superstructure.pivot.Pivot
import frc.team449.subsystems.superstructure.wrist.Wrist
import frc.team449.subsystems.superstructure.wrist.WristConstants

class SuperstructureManager(
  private val elevator: Elevator,
  private val pivot: Pivot,
  private val wrist: Wrist,
  private val drive: SwerveDrive
) {

  var lastRequestedGoal = SuperstructureGoal.STOW

  fun requestGoal(goal: SuperstructureGoal.SuperstructureState): Command {
    return InstantCommand({ SuperstructureGoal.applyDriveDynamics(drive, goal.driveDynamics) })
      .andThen(InstantCommand({ lastRequestedGoal = goal }))
      .andThen(
        ConditionalCommand(
          // if extending
          Commands.sequence(
            Commands.parallel(
              wrist.setPosition(goal.wrist.`in`(Radians)),
              pivot.setPosition(goal.pivot.`in`(Radians))
            ),
            WaitUntilCommand { wrist.elevatorReady() },
            elevator.setPosition(goal.elevator.`in`(Meters)),
            WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() },
            Commands.parallel(pivot.hold(), wrist.hold())
              .repeatedly()
              .until { elevator.atSetpoint() },
            holdAll()
          ),

          // if retracting
          Commands.sequence(
            elevator.setPosition(goal.elevator.`in`(Meters)),
            wrist.hold(),
            wrist.setPosition(WristConstants.ELEVATOR_READY.`in`(Radians))
              .onlyIf { goal.wrist > WristConstants.ELEVATOR_READY },
            pivot.hold()
              .repeatedly()
              .until { elevator.atSetpoint() },
            Commands.parallel(
              pivot.setPosition(goal.pivot.`in`(Radians)),
              wrist.setPosition(goal.wrist.`in`(Radians))
            ),
            elevator.hold()
              .repeatedly()
              .until { wrist.atSetpoint() && pivot.atSetpoint() },
            holdAll()
          )
        ) { goal.elevator.`in`(Meters) >= elevator.positionSupplier.get() }
      )
  }

  private fun holdAll(): Command {
    return Commands.parallel(
      pivot.hold(),
      wrist.hold(),
      elevator.hold()
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
