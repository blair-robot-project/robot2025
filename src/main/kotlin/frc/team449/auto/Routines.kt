package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import choreo.trajectory.SwerveSample
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.Robot
import frc.team449.subsystems.superstructure.SuperstructureGoal
import kotlin.jvm.optionals.getOrDefault
import kotlin.math.PI

open class Routines(
  val robot: Robot
) {
  private val xController: PIDController
    get() = PIDController(10.0, 0.0, 0.0)
  private val yController: PIDController
    get() = PIDController(10.0, 0.0, 0.0)
  private val headingController: PIDController
    get() = PIDController(6.7, 0.0, 0.0)

  init {
    headingController.enableContinuousInput(-Math.PI, Math.PI)
  }

  private fun followTrajectory(sample: SwerveSample) {
    val speeds = ChassisSpeeds(
      sample.vx + xController.calculate(robot.poseSubsystem.pose.x, sample.x),
      sample.vy + yController.calculate(robot.poseSubsystem.pose.y, sample.y),
      sample.omega + headingController.calculate(
        robot.poseSubsystem.pose.rotation.radians + if (DriverStation.getAlliance().getOrDefault(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) PI else 0.0,
        sample.heading
      )
    )
    val newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, robot.poseSubsystem.heading)
    // Apply the generated speeds
    robot.drive.driveRobotRelative(newSpeeds)
  }

  val autoFactory = AutoFactory(
    robot.poseSubsystem::pose,
    robot.poseSubsystem::resetPoseChoreo,
    { sample: SwerveSample -> followTrajectory(sample) },
    true,
    robot.drive
  )

  fun americanRoutine(): AutoRoutine {
    val autoRoutine: AutoRoutine = autoFactory.newRoutine("L4 Routine")


    val l4fTrajectory: AutoTrajectory = autoRoutine.trajectory("Go To L4F")
    val rightStationTrajectory: AutoTrajectory = autoRoutine.trajectory("Go To Right Station(2)")
    val l1CTrajectory: AutoTrajectory = autoRoutine.trajectory("Go To L1C")
    val rightStationTrajectory2: AutoTrajectory = autoRoutine.trajectory("Go To Right Station(Again)")
    val l1bTrajectory:AutoTrajectory=autoRoutine.trajectory("Go To L1B")
    val leftStationTrajectory:AutoTrajectory=autoRoutine.trajectory("Go To Left Station")
    val l4kTrajectory:AutoTrajectory=autoRoutine.trajectory("Go To L4K")

    autoRoutine.active().onTrue(
      Commands.sequence(
        l4fTrajectory.resetOdometry(),
        l4fTrajectory.cmd(),
        PrintCommand("Traveling to l4"),
        ),
      )

    l4fTrajectory.done().onTrue(rightStationTrajectory.cmd().
    beforeStarting(robot.superstructureManager.requestGoal(SuperstructureGoal.L4_fromStation).
    beforeStarting(robot.drive.driveStop())))

    rightStationTrajectory.done().onTrue(l1CTrajectory.cmd().
    beforeStarting(robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)
      .beforeStarting(robot.drive.driveStop())))

    l1CTrajectory.done().onTrue(rightStationTrajectory2.cmd().
    beforeStarting(robot.superstructureManager.requestGoal(SuperstructureGoal.L1).
    beforeStarting(robot.drive.driveStop())))

    rightStationTrajectory2.done().onTrue(l1bTrajectory.cmd().
    beforeStarting(robot.drive.driveStop()).
    andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.SUBSTATION_INTAKE)))

    l1bTrajectory.done().onTrue(leftStationTrajectory.cmd().beforeStarting(robot.drive.driveStop())
      .andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.L1_fromStation)))

    leftStationTrajectory.done().onTrue(l4kTrajectory.cmd()
      .beforeStarting(robot.superstructureManager.requestGoal(SuperstructureGoal.L4)).
      beforeStarting(robot.drive.driveStop()))

    l4kTrajectory.done().onTrue(robot.drive.driveStop().andThen(robot.superstructureManager.requestGoal(SuperstructureGoal.L1)))




    return autoRoutine
  }




  // autoChooser that will be displayed on dashboard
  fun addOptions(autoChooser: AutoChooser) {

    autoChooser.addRoutine("routine",this::americanRoutine)
  }
}
