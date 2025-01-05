// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.team449.subsystems.vision.interpolation

import edu.wpi.first.math.geometry.Translation2d
import kotlin.math.pow

object InterpolationUtil {
  private const val USE_LINEAR_WEIGHTS = false
  fun interpolateTranslation(
    dataPoints: List<VisionInterpolationData>,
    visionInput: Translation2d
  ): Translation2d {
    val unnormalizedWeightsSum = dataPoints.stream()
      .mapToDouble { (_, visionPose): VisionInterpolationData -> calculateUnnormalizedWeight(visionPose.getDistance(visionInput)) }
      .sum()
    var weightedX = 0.0
    var weightedY = 0.0
    for ((measuredPose, visionPose) in dataPoints) {
      val distancePoint = visionPose.getDistance(visionInput)
      val weight = calculateUnnormalizedWeight(distancePoint) / unnormalizedWeightsSum
      val result = measuredPose.minus(visionPose).times(weight)
      weightedX += result.x
      weightedY += result.y
    }
    return Translation2d(visionInput.x + weightedX, visionInput.y + weightedY)
  }

  private fun calculateUnnormalizedWeight(distance: Double): Double {
    return if (USE_LINEAR_WEIGHTS) {
      (1.0 + distance) / distance
    } else {
      1.0 / distance.pow(2.0)
    }
  }
}
