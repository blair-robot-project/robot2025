// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.team449.subsystems.vision.interpolation

import edu.wpi.first.math.geometry.Translation2d

enum class InterpolatedVisionDataset(
    red: List<VisionInterpolationData>,
    blue: List<VisionInterpolationData>,
) {
    HOMELEFT(
        listOf(
            VisionInterpolationData(
                Translation2d(15.2245, 5.522),
                Translation2d(15.2245, 5.522),
                "RED_SUBWOOFER",
            )
            // continue for more places
        ),
        listOf(
            VisionInterpolationData(
                Translation2d(1.315, 5.522),
                Translation2d(1.39, 5.53),
                "BLUE_SUBWOOFER",
            )
            // continue for more
        ),
    ),
    HOMERIGHT(
        listOf(
            VisionInterpolationData(
                Translation2d(15.2245, 5.522),
                Translation2d(15.2245, 5.522),
                "RED_SUBWOOFER",
            )
            // continue for more places
        ),
        listOf(
            VisionInterpolationData(
                Translation2d(1.315, 5.522),
                Translation2d(1.39, 5.53),
                "BLUE_SUBWOOFER",
            )
        ),
    );

    val redSet: List<VisionInterpolationData>
    val blueSet: List<VisionInterpolationData>

    init {
        redSet = red
        blueSet = blue
    }
}
