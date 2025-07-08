package frc.team449.subsystems.superstructure

import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.Robot
import frc.team449.subsystems.drive.swerve.SwerveDrive
import frc.team449.subsystems.superstructure.elevator.Elevator
import frc.team449.subsystems.superstructure.pivot.Pivot
import frc.team449.subsystems.superstructure.wrist.Wrist
import frc.team449.subsystems.superstructure.wrist.WristConstants
import frc.team449.subsystems.vision.PoseSubsystem

class SuperstructureManager(
  private val elevator: Elevator,
  private val pivot: Pivot,
  private val wrist: Wrist,
  private val drive: SwerveDrive,
  private val poseSubsystem: PoseSubsystem
) {

  private var requestedGoal = SuperstructureGoal.STOW
  private var lastCompletedGoal = SuperstructureGoal.STOW
  private var ready = false
  private val timer = Timer()

  private fun handleCoralToAlgaeGround(): Command {
    val goal = SuperstructureGoal.ALGAE_GROUND
    // move wrist forward with pivot to setpoint and then move wrist to setpoint
    return Commands.sequence(
      pivot.setPosition(goal.pivot.`in`(Radians))
        // move wrist 30 degrees forward to prevent crashing
        .alongWith(wrist.setPosition(wrist.positionSupplier.get() + 0.5)),
      // wait until pivot is almost at setpoint before moving wrist
      WaitUntilCommand { pivot.atSetpoint(Degrees.of(6.0)) },
      wrist.setPosition(goal.wrist.`in`(Radians)),
      WaitUntilCommand { wrist.atSetpoint() || pivot.atSetpoint() },
      pivot.hold().onlyIf { pivot.atSetpoint() },
      wrist.hold().onlyIf { wrist.atSetpoint() },
      WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() },
      holdAll()
    )
  }

  private fun handleAlgaeToCoralGround(): Command {
    val goal = SuperstructureGoal.GROUND_INTAKE_CORAL
    // move wrist before moving pivot
    return Commands.sequence(
      wrist.setPosition(goal.wrist.`in`(Radians)),
      // wait until pivot is almost at setpoint before moving wrist
      WaitUntilCommand { wrist.atSetpoint(Units.degreesToRadians(5.0)) },
      pivot.setPosition(goal.pivot.`in`(Radians)),
      WaitUntilCommand { wrist.atSetpoint() || pivot.atSetpoint() },
      pivot.hold().onlyIf { pivot.atSetpoint() },
      wrist.hold().onlyIf { wrist.atSetpoint() },
      WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() },
      holdAll()
    )
  }

  private fun handleExtension(goal: SuperstructureGoal.SuperstructureState): Command {
    return Commands.sequence(
      Commands.parallel(
        wrist.setPosition(goal.wrist.`in`(Radians)),
        pivot.setPosition(goal.pivot.`in`(Radians))
      ),
      WaitUntilCommand { wrist.atSetpoint() || pivot.atSetpoint() },
      pivot.hold().onlyIf { pivot.atSetpoint() },
      wrist.hold().onlyIf { wrist.atSetpoint() },

      WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() },
      pivot.hold(),
      wrist.hold(),
      elevator.setPosition(goal.elevator.`in`(Meters)),

      WaitUntilCommand { elevator.atSetpoint() },
      holdAll()
    )
  }

  private fun requestExtension(goal: SuperstructureGoal.SuperstructureState): Command {
    return Commands.sequence(

      ConditionalCommand( // going from coral to algae ground comparison

        // if we're going from CORAL GROUND to ALGAE GROUND (elevator the same)
        handleCoralToAlgaeGround(),

        // we're not going to coral ground from algae ground
        ConditionalCommand( // going from algae ground to coral ground comparison

          // if we're going from ALGAE GROUND to CORAL GROUND (elevator the same)
          handleAlgaeToCoralGround(),

          // regular extension
          handleExtension(goal)

        ) { goal == SuperstructureGoal.GROUND_INTAKE_CORAL && lastCompletedGoal == SuperstructureGoal.ALGAE_GROUND }

      ) { goal == SuperstructureGoal.ALGAE_GROUND && lastCompletedGoal == SuperstructureGoal.GROUND_INTAKE_CORAL }
    )
  }

  // retract from front side scoring
  private fun retractFromL4(goal: SuperstructureGoal.SuperstructureState = SuperstructureGoal.STOW): Command {
    return Commands.sequence(
      wrist.setPosition(Units.degreesToRadians(60.0)),
      WaitUntilCommand { wrist.positionSupplier.get() > Units.degreesToRadians(50.0) },

      elevator.setPosition(goal.elevator.`in`(Meters)),
      WaitUntilCommand { elevator.pivotReady() },

      wrist.setPosition(goal.wrist.`in`(Radians)),
      pivot.setPosition(goal.pivot.`in`(Radians)),
      WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() && elevator.atSetpoint() },
      holdAll()
    )
  }

  private fun handleRetraction(goal: SuperstructureGoal.SuperstructureState): Command {
    return Commands.sequence(

      elevator.setPosition(goal.elevator.`in`(Meters)),
      wrist.setPosition(WristConstants.ELEVATOR_READY.`in`(Radians))
        .onlyIf { goal.wrist > WristConstants.ELEVATOR_READY },

      WaitUntilCommand { elevator.pivotReady() },

      pivot.setPosition(goal.pivot.`in`(Radians)),
      wrist.setPosition(goal.wrist.`in`(Radians)),

      WaitUntilCommand { wrist.atSetpoint() || pivot.atSetpoint() },
      pivot.hold().onlyIf { pivot.atSetpoint() },
      wrist.hold().onlyIf { wrist.atSetpoint() },

      WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() },
      pivot.hold(),
      wrist.hold(),

      WaitUntilCommand { elevator.atSetpoint() },
      holdAll()
    )
  }

  // retracting from l4 pivot side and l3 pivot side
  private fun retractFromPivot(goal: SuperstructureGoal.SuperstructureState): Command {
    return Commands.sequence(

      wrist.setPosition(Units.degreesToRadians(60.0)),
      WaitUntilCommand { wrist.positionSupplier.get() < Units.degreesToRadians(70.0) },

      elevator.setPosition(goal.elevator.`in`(Meters)), // about 0.15 meter
      WaitCommand(0.2),

      pivot.setPosition(pivot.positionSupplier.get() - 0.045).onlyIf {
        lastCompletedGoal == SuperstructureGoal.L4_PIVOT
      },

      WaitUntilCommand { pivot.atSetpoint() || elevator.atSetpoint() },
      elevator.hold().onlyIf { elevator.atSetpoint() },
      pivot.hold().onlyIf { pivot.atSetpoint() },

      WaitUntilCommand { pivot.atSetpoint() && elevator.atSetpoint() },
      pivot.hold(),
      elevator.hold(),

      pivot.setPosition(goal.pivot.`in`(Radians)),

      elevator.setPosition(goal.elevator.`in`(Meters)),
      wrist.setPosition(goal.wrist.`in`(Radians)),

      WaitUntilCommand { pivot.atSetpoint() },
      pivot.hold(),

      WaitUntilCommand { wrist.atSetpoint() || elevator.atSetpoint() },
      elevator.hold().onlyIf { elevator.atSetpoint() },
      wrist.hold().onlyIf { wrist.atSetpoint() },

      WaitUntilCommand { wrist.atSetpoint() && elevator.atSetpoint() },
      holdAll()

    )
  }

  fun requestRetraction(goal: SuperstructureGoal.SuperstructureState): Command {
    return ConditionalCommand( // previous goal needs special retraction case

      ConditionalCommand( // pivot side retraction comparison

        // if we scored l4 or l3 pivot side we need to watch out for climb
        retractFromPivot(goal),

        // if not pivot side then l4 retraction
        retractFromL4(goal)

      ) { requestedPivotSide() },

      // regular retraction
      handleRetraction(goal)

    ) {
      lastCompletedGoal == SuperstructureGoal.L4 ||
        lastCompletedGoal == SuperstructureGoal.L4_PIVOT ||
        lastCompletedGoal == SuperstructureGoal.L3_PIVOT
    }
  }

  private fun requestHigh(goal: SuperstructureGoal.SuperstructureState = SuperstructureGoal.L4): Command {
    return ConditionalCommand(
      // if extending
      Commands.sequence(
        Commands.parallel(
          wrist.setPosition(goal.wrist.`in`(Radians)),
          pivot.setPosition(goal.pivot.`in`(Radians))
        ),
        WaitUntilCommand { pivot.elevatorReady() },
        elevator.setPositionCarriage(goal.elevator.`in`(Meters)),
        WaitUntilCommand { wrist.atSetpoint() || pivot.atSetpoint() },
        pivot.hold().onlyIf { pivot.atSetpoint() },
        wrist.hold().onlyIf { wrist.atSetpoint() },
        WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() },
        pivot.hold(),
        wrist.hold(),
        WaitUntilCommand { elevator.atSetpoint() },
        Commands.parallel(
          pivot.hold(),
          wrist.hold(),
          elevator.holdCarriage()
        )
      ),

      // if retracting
      Commands.sequence(
        wrist.hold(),
        elevator.setPositionCarriage(goal.elevator.`in`(Meters)),
        wrist.setPosition(WristConstants.ELEVATOR_READY.`in`(Radians))
          .onlyIf { goal.wrist > WristConstants.ELEVATOR_READY },
        WaitUntilCommand { elevator.pivotReady() },
        Commands.parallel(
          pivot.setPosition(goal.pivot.`in`(Radians)),
          wrist.setPosition(goal.wrist.`in`(Radians))
        ),
        WaitUntilCommand { wrist.atSetpoint() && pivot.atSetpoint() && elevator.atSetpoint() },
        Commands.parallel(
          pivot.hold(),
          wrist.hold(),
          elevator.holdCarriage()
        )
      )
    ) { goal.elevator.`in`(Meters) >= elevator.positionSupplier.get() }
  }

  fun requestGoal(goal: SuperstructureGoal.SuperstructureState): Command {
    return InstantCommand({ SuperstructureGoal.applyDriveDynamics(drive, goal.driveDynamics) })
      .andThen(InstantCommand({ ready = false }))
      .andThen(InstantCommand({ requestedGoal = goal }))
      .andThen(
        runOnce({
          timer.reset()
        })
      )
      .andThen(

        ConditionalCommand( // goal is high comparison
          // use special function for high goals
          requestHigh(goal),

          // not a high goal
          ConditionalCommand( // elevator height comparison

            // if extending or elevator the same
            requestExtension(goal),

            // if retracting
            requestRetraction(goal)

          ) { goal.elevator.`in`(Meters) >= elevator.positionSupplier.get() }

        ) {
          goal == SuperstructureGoal.L4 || goal == SuperstructureGoal.L4_PIVOT ||
            goal == SuperstructureGoal.NET || goal == SuperstructureGoal.NET_PIVOT
        }

      )
      .andThen(runOnce({ timer.reset() }))
      .andThen(InstantCommand({ lastCompletedGoal = goal }))
      .andThen(InstantCommand({ ready = true }))
//    {
//        FieldConstants.FIELD_CONFIGURED &&
//
//        ( goal == SuperstructureGoal.ALGAE_GROUND ||
//          goal == SuperstructureGoal.GROUND_INTAKE_CORAL ) &&
//
//        poseSubsystem.pose.translation.getDistance(
//          poseSubsystem.pose.nearest(FieldConstants.REEF_LOCATIONS).translation
//        ) < FieldConstants.DIST_FOR_SAFE_GI
//
//        && !poseSubsystem.isPivotSide()
//    }
  }

  fun isAtPos(): Boolean {
    return ready
  }

  fun lastRequestedGoal(): SuperstructureGoal.SuperstructureState {
    return requestedGoal
  }

  fun requestedPivotSide(): Boolean {
    return lastCompletedGoal == SuperstructureGoal.L2_PIVOT ||
      lastCompletedGoal == SuperstructureGoal.L3_PIVOT ||
      lastCompletedGoal == SuperstructureGoal.L4_PIVOT
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
        robot.drive,
        robot.poseSubsystem
      )
    }
  }
}
