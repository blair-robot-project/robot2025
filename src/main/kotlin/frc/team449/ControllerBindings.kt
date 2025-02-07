package frc.team449

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
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

  private val autoscore = AutoScoreCommands(robot.drive, robot.poseSubsystem, robot.driveController.hid, robot)

  private fun robotBindings() {
//    println("configuring the drive")
//    AutoBuilder.configure(
//      robot.poseSubsystem::getPosea, // poseSupplier - a supplier for the robot's current pose
//      robot.poseSubsystem::resetOdometry, // resetPose - a consumer for resetting the robot's pose
//      robot.drive::getCurrentSpeedsa, // robotRelativeSpeedsSupplier - a supplier for the robot's current robot relative chassis speeds
//      robot.drive::set, // output - Output function that accepts robot-relative ChassisSpeeds
//      PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
//        PIDConstants(5.0, 0.0, 0.0), // Translation PID constants, placeholders
//        PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants, placeholders
//      ),
//      RobotConfig.fromGUISettings(),
//      { DriverStation.getAlliance().get() == Alliance.Red },
//      robot.drive // driveRequirements - the subsystem requirements for the robot's drive train
//    )
//
//    println("drive configured")
    // reef location passed in alla webappp, this is temp
//    robot.driveController.x().onTrue(
//      PrintCommand("moving to reef").andThen(
//        autoscore.moveToReefCommand(AutoScoreCommandConstants.ReefLocation.Location1)
//      ).andThen(PrintCommand("reef finished"))
//    )
//    robot.driveController.a().onTrue(
//      PrintCommand("move to processor button pressed").andThen(
//      autoscore.moveToProcessorCommand()).andThen(PrintCommand("moved to processor")).andThen(autoscore.scoreProcessorCommand()).andThen(PrintCommand("processor scored"))
//    )
//    //on red alliance side passed in by webapp, this is temp
//    robot.driveController.b().onTrue(
//      PrintCommand("moving to net").andThen(
//        autoscore.moveToNetCommand(false)
//      ).andThen(PrintCommand("net finished"))
//    )
//    robot.driveController.y().onTrue(PrintCommand("move to coral intake button pressed").andThen(
//      autoscore.moveToCoralIntakeCommand(true)).andThen(PrintCommand("moved to coral intake"))
//        .andThen(autoscore.intakeCoralCommand()).andThen(PrintCommand("coral intaken"))
//    )

//    println("pose from robot loop ${robot.poseSubsystem.getPosea()}")
    robot.driveController.x().onTrue(robot.pathfinder.path(AutoScoreCommandConstants.reef1PoseBlue))

    //robot.driveController.x().onTrue(PrintCommand("invoke path").andThen(robot.pathfinder.path(AutoScoreCommandConstants.reef1PoseBlue)))
    //robot.driveController.a().onTrue(PrintCommand("invoke other path").andThen(robot.pathfinder.path(AutoScoreCommandConstants.reef1PoseRed)))
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
//    driveController.a().onTrue(
//      robot.driveCommand.pointAtAngleCommand(Rotation2d.fromDegrees(90.0))
//    )
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
