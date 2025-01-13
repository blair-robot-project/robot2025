// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.team449.subsystems.vision.interpolation

import edu.wpi.first.math.geometry.Translation2d

data class VisionInterpolationData(val measuredPose: Translation2d, val visionPose: Translation2d, val label: String)
