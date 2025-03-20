// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.team449.subsystems.vision.interpolation

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation
import frc.team449.subsystems.vision.VisionConstants
import kotlin.jvm.optionals.getOrNull

object InterpolatedVision {
  private val usedSet: List<InterpolatedVisionDataset> = VisionConstants.interpolatedVisionSets

  /**
   * @param visionInput - pose from the limelight
   * @return a transformed pose that can be added to the pose estimator
   */
  fun interpolatePose(visionInput: Pose2d, camIndex: Int): Pose2d {
    return if (usedSet.isNotEmpty()) {
      val usedDataPoints = if (DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red) usedSet[camIndex].redSet else usedSet[camIndex].blueSet

      Pose2d(
        InterpolationUtil.interpolateTranslation(usedDataPoints, visionInput.translation),
        visionInput.rotation
      )
    } else {
      visionInput
    }
  }
}
