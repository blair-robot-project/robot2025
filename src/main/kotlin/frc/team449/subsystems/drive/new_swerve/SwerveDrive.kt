package frc.team449.subsystems.drive.new_swerve

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj2.command.Subsystem


interface SwerveDrive: Subsystem{
  fun drive(speeds: ChassisSpeeds, fieldRelative: Boolean, isOpenLoop: Boolean)
  fun setModuleStates(desiredStates: Array<SwerveModuleState>)
  fun getMeasuredSpeeds(): ChassisSpeeds
  fun getGyroYaw(): Rotation2d
  fun getPose(): Pose2d
  fun setPose(pose: Pose2d)
  fun getHeading(): Rotation2d {
    return getPose().rotation
  }
  fun setHeading(heading: Rotation2d){
    setPose(Pose2d(getPose().translation, heading))
  }
  fun zeroHeading(){
    setHeading(Rotation2d())
  }
  fun addVisionMeasurement(visionRobotPose: Pose2d, timeStampSeconds: Float){}
  fun addVisionMeasurement(visionRobotPoseMeters: Pose2d, timeStampSeconds: Float, visionMeasurementStdDevs: Matrix<N3, N1>){

  }
}