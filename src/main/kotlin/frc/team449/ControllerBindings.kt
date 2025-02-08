package frc.team449

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.commands.autoscoreCommands.AutoScoreCommandConstants
import frc.team449.commands.autoscoreCommands.AutoScoreCommands
import frc.team449.subsystems.RobotConstants
import frc.team449.subsystems.drive.swerve.SwerveSim
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI
import kotlin.random.Random

class ControllerBindings(
  private val driveController: CommandXboxController,
  private val mechanismController: CommandXboxController,
  private val robot: Robot
) {

  private val autoScore = AutoScoreCommands(robot.poseSubsystem, robot)

  private fun robotBindings() {
    AutoBuilder.configure(
      robot.poseSubsystem::getPosea,
      robot.poseSubsystem::resetOdometry,
      robot.drive::currentSpeeds,
      robot.poseSubsystem::edemPathMag,
      PPHolonomicDriveController(
        PIDConstants(5.0, 0.0, 0.0),
        PIDConstants(5.0, 0.0, 0.0)
      ),
      RobotConfig.fromGUISettings(),
      { DriverStation.getAlliance().get() == Alliance.Red },
      robot.drive
    )

    // these testing commands are just temp, wont be on controller
    robot.driveController.x().onTrue(
      runOnce({
        autoScore.currentCommand = autoScore.reef(AutoScoreCommandConstants.ReefLocation.Location2, AutoScoreCommandConstants.ReefLevel.L1)
        autoScore.poseSubsystem.autoscoreCurrentCommand = autoScore.currentCommand
        autoScore.currentCommand.schedule()
      })
    )
    robot.driveController.a().onTrue(
      runOnce({
        autoScore.currentCommand = autoScore.processor()
        autoScore.poseSubsystem.autoscoreCurrentCommand = autoScore.currentCommand
        autoScore.currentCommand.schedule()
      })
    )
    // on red alliance side passed in by webapp, this is temp
    robot.driveController.b().onTrue(
      runOnce({
        autoScore.currentCommand = autoScore.net(true)
        autoScore.poseSubsystem.autoscoreCurrentCommand = autoScore.currentCommand
        autoScore.currentCommand.schedule()
      })
    )
    robot.driveController.y().onTrue(
      runOnce({
        autoScore.currentCommand.cancel()
      })
    )
//    var reefPose = AutoScoreCommandConstants.testPose
//    val constraints = PathConstraints(
//      3.0,
//      4.0,
//      Units.degreesToRadians(540.0),
//      Units.degreesToRadians(720.0)
//    )
//    robot.driveController.x().onTrue(
//      AutoBuilder.pathfindToPose(
//        reefPose,
//        constraints,
//        0.0,
//        // Goal end velocity in meters/sec
//      )
//    )

//    robot.driveController.x().onTrue(autoscore.magnetizeToTestCommand())

//    robot.driveController.x().onTrue(autoscore.moveToReefCommand(AutoScoreCommandConstants.ReefLocation.Location1))

    /** Call robot functions you create below */
//    driveController.a().onTrue(
//      robot.superstructureManager.requestGoal(SuperstructureGoal.L1)
//    )
//
//    driveController.b().onTrue(
//      robot.superstructureManager.requestGoal(SuperstructureGoal.STOW)
//    )
  }

  private fun nonRobotBindings() {
    // slowDrive()

    if (RobotBase.isSimulation()) resetOdometrySim()

    resetGyro()
  }

  private fun slowDrive() {
    driveController.rightBumper().onTrue(
      InstantCommand({ robot.drive.maxLinearSpeed = 1.0 })
        .andThen(InstantCommand({ robot.drive.maxRotSpeed = PI / 2 }))
    ).onFalse(
      InstantCommand({ robot.drive.maxLinearSpeed = RobotConstants.MAX_LINEAR_SPEED })
        .andThen(
          InstantCommand({ robot.drive.maxRotSpeed = RobotConstants.MAX_ROT_SPEED })
        )
    )
  }

  private fun resetOdometrySim() {
    driveController.a().onTrue(
      InstantCommand({
        robot.drive as SwerveSim
        robot.drive.resetOdometryOnly(
          Pose2d(
            robot.drive.odometryPose.x + Random.nextDouble(-1.0, 1.0),
            robot.drive.odometryPose.y + Random.nextDouble(-1.0, 1.0),
            robot.drive.odometryPose.rotation
          )
        )
      })
    )
  }
  private fun resetGyro() {
    driveController.start().onTrue(
      ConditionalCommand(
        InstantCommand({ robot.poseSubsystem.heading = Rotation2d(PI) }),
        InstantCommand({ robot.poseSubsystem.heading = Rotation2d() })
      ) { DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red }
    )
  }

  private fun pointToRight() {
    driveController.a().onTrue(
      robot.driveCommand.pointAtAngleCommand(Rotation2d.fromDegrees(90.0))
    )
  }

  /** Characterization functions */
  private fun characterizationExample() {
    /** Example
     *
     val exampleSubsystemRoutine = SysIdRoutine(
     SysIdRoutine.Config(
     Volts.of(0.5).per(Seconds.of(1.0)),
     Volts.of(3.0),
     Seconds.of(10.0)
     ) { state -> SignalLogger.writeString("state", state.toString()) },
     Mechanism(
     { voltage: Measure<Voltage> ->
     run { robot.shooter.setVoltage(voltage.`in`(Volts)) }
     },
     null,
     robot.shooter,
     "shooter"
     )
     )

     // Quasistatic Forwards
     driveController.povUp().onTrue(
     exampleSubsystemRoutine.quasistatic(SysIdRoutine.Direction.kForward)
     )

     // Quasistatic Reverse
     driveController.povDown().onTrue(
     exampleSubsystemRoutine.quasistatic(SysIdRoutine.Direction.kReverse)
     )

     // Dynamic Forwards
     driveController.povRight().onTrue(
     exampleSubsystemRoutine.dynamic(SysIdRoutine.Direction.kForward)
     )

     // Dynamic Reverse
     driveController.povLeft().onTrue(
     exampleSubsystemRoutine.dynamic(SysIdRoutine.Direction.kReverse)
     )
     */
  }

  /** Try not to touch, just add things to the robot or nonrobot bindings */
  fun bindButtons() {
    nonRobotBindings()
    robotBindings()
  }
}
