package frc.team449.subsystems.drive.new_swerve

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.Subsystem


interface SwerveDrive: Subsystem{
  fun drive(speeds: ChassisSpeeds, fieldRelative: Boolean, isOpenLoop: Boolean)
  fun setModuleStates(desiredStates: Array<SwerveModuleState>)
  fun getMeasuredSpeeds(): ChassisSpeeds
  fun getGyroYaw(): Rotation2d
  fun getPose(): Pose2d
  fun setPose(pose: Pose2d)
  fun getHeading(){
    return getPose().getRotation();
  }
  fun setHeading(heading: Rotation2d){
    setPose(Pose2d(getPose().getTranslation(), heading))
  }
  fun zeroHeading(){
    setHeading(Rotation2d())
  }
  fun addVisionMeasurement(visionRobotPose: Pose2d, timeStampSeconds: float){}
  fun addVisionMeasurement(visionRobotPoseMeters: Pose2d, timeStampSeconds: float, visionMeasurementStdDevs: Matrix<N3, N1>){

  }
}