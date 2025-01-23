package frc.team449.subsystems.drive.swerve

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team449.subsystems.drive.new_swerve.SwerveDrive
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig

open class MapleSimSwerve : SwerveDrive {
  private val simulatedDrive: SelfControlledSwerveDriveSimulation
  private val field2d: Field2d

  init {
    // For your own code, please configure your drivetrain properly according to the documentation
    val config = DriveTrainSimulationConfig.Default()

    // Creating the SelfControlledSwerveDriveSimulation instance
    simulatedDrive = SelfControlledSwerveDriveSimulation(
      SwerveDriveSimulation(config, Pose2d(0.0, 0.0, Rotation2d()))
    )

    // Register the drivetrain simulation to the simulation world
    SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.driveTrainSimulation)

    // A field2d widget for debugging
    field2d = Field2d()
    SmartDashboard.putData("simulation field", field2d)
  }

  fun drive(translation : Translation2d, rotation : Double, fieldRelative : Boolean, isOpenLoop : Boolean) {
    this.simulatedDrive.runChassisSpeeds(
      ChassisSpeeds(translation.x, translation.y, rotation),
      Translation2d(),
      fieldRelative,
    true);
  }

  override fun drive(speeds: ChassisSpeeds, fieldRelative: Boolean, isOpenLoop: Boolean) {
    TODO("Not yet implemented")
  }

  override fun setModuleStates(desiredStates : Array<SwerveModuleState>  ) {
    simulatedDrive.runSwerveStates(desiredStates);
  }

  override fun getMeasuredSpeeds() : ChassisSpeeds {
    return simulatedDrive.getMeasuredSpeedsFieldRelative(true);
  }

  override fun getGyroYaw() : Rotation2d {
    return simulatedDrive.getRawGyroAngle();
  }

  override fun getPose() : Pose2d {
    return simulatedDrive.odometryEstimatedPose;
  }

  override fun setPose(pose : Pose2d) {
    simulatedDrive.setSimulationWorldPose(pose);
    simulatedDrive.resetOdometry(pose);
  }

  override fun addVisionMeasurement(visionRobotPose : Pose2d, timeStampSeconds : Double) {
    simulatedDrive.addVisionEstimation(visionRobotPose, timeStampSeconds);
  }

  override fun addVisionMeasurement(
    visionRobotPoseMeters : Pose2d, timeStampSeconds : Double, visionMeasurementStdDevs : Matrix<N3, N1>) {
    simulatedDrive.addVisionEstimation(visionRobotPoseMeters, timeStampSeconds, visionMeasurementStdDevs);
  }

  override fun periodic() {
    // update the odometry of the SimplifedSwerveSimulation instance
    simulatedDrive.periodic();

    // send simulation data to dashboard for testing
    field2d.robotPose = simulatedDrive.actualPoseInSimulationWorld;
    field2d.getObject("odometry").pose = getPose();
  }

}
