package frc.team449.auto

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Nat
import edu.wpi.first.math.controller.PIDController
import frc.team449.auto.choreo.ChoreoTrajectory
import frc.team449.subsystems.FieldConstants
import frc.team449.subsystems.RobotConstants
import kotlin.math.PI

object AutoUtil {

    /**
     * If you need snap to angle during auto (for vision assisted pointing), use this controller
     * since the orthogonal command is not set up yet during auto
     */
    val turnController =
        PIDController(RobotConstants.SNAP_KP, RobotConstants.SNAP_KI, RobotConstants.SNAP_KD)

    /** You will need to change this based on how the field is set up each year */
    fun transformForRed(pathGroup: MutableList<ChoreoTrajectory>): MutableList<ChoreoTrajectory> {
        for (index in 0 until pathGroup.size) {
            for (time in pathGroup[index].objectiveTimestamps) {
                val currentMatrix = pathGroup[index].stateMap.get(time)

                val newMatrix =
                    MatBuilder.fill(
                        Nat.N2(),
                        Nat.N3(),
                        FieldConstants.fieldLength - currentMatrix[0, 0],
                        currentMatrix[0, 1],
                        MathUtil.angleModulus(PI - currentMatrix[0, 2]),
                        -currentMatrix[1, 0],
                        currentMatrix[1, 1],
                        -currentMatrix[1, 2],
                    )

                pathGroup[index].stateMap.put(time, newMatrix)
            }
        }

        return pathGroup
    }

    /**
     * Add other methods that return commands that do groups of actions that are done across
     * different auto routines. For Charged UP, these methods were things such as dropping a
     * cone/cube, or getting in ground intake position, etc.
     */
}
