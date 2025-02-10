package com.pathplanner.lib.pathfinding

import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.path.PathPoint
import edu.wpi.first.math.Pair
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.networktables.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import java.io.BufferedReader
import java.io.File
import java.io.FileReader
import java.util.EnumSet
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicReference

/** Implementation of ADStar running on a coprocessor  */
class RemoteADStar : Pathfinder {
  private val navGridJsonPub: StringPublisher
  private val startPosPub: DoubleArrayPublisher
  private val goalPosPub: DoubleArrayPublisher
  private val dynamicObsPub: DoubleArrayPublisher

  private val pathPointsSub: DoubleArraySubscriber

  private val currentPath = AtomicReference<List<PathPoint>>(ArrayList())
  private val newPathAvailable = AtomicBoolean(false)

  /** Create a RemoteADStar object. This will use NT4 to communicate with the coprocessor.  */
  init {
    val nt = NetworkTableInstance.getDefault()

    navGridJsonPub = nt.getStringTopic("/PPLibCoprocessor/RemoteADStar/navGrid").publish()
    startPosPub = nt.getDoubleArrayTopic("/PPLibCoprocessor/RemoteADStar/startPos").publish()
    goalPosPub = nt.getDoubleArrayTopic("/PPLibCoprocessor/RemoteADStar/goalPos").publish()
    dynamicObsPub =
      nt.getDoubleArrayTopic("/PPLibCoprocessor/RemoteADStar/dynamicObstacles").publish()

    pathPointsSub =
      nt.getDoubleArrayTopic("/PPLibCoprocessor/RemoteADStar/pathPoints")
        .subscribe(
          DoubleArray(0),
          PubSubOption.keepDuplicates(true),
          PubSubOption.sendAll(true)
        )

    nt.addListener(
      pathPointsSub,
      EnumSet.of(NetworkTableEvent.Kind.kValueAll)
    ) { event: NetworkTableEvent? ->
      val pathPointsArr = pathPointsSub.get()
      val pathPoints: MutableList<PathPoint> = ArrayList()
      var i = 0
      while (i <= pathPointsArr.size - 2) {
        pathPoints.add(
          PathPoint(Translation2d(pathPointsArr[i], pathPointsArr[i + 1]), null)
        )
        i += 2
      }

      currentPath.set(pathPoints)
      newPathAvailable.set(true)
    }

    val navGridFile = File(Filesystem.getDeployDirectory(), "pathplanner/navgrid.json")
    if (navGridFile.exists()) {
      try {
        BufferedReader(FileReader(navGridFile)).use { br ->
          val fileContentBuilder = StringBuilder()
          var line: String?
          while ((br.readLine().also { line = it }) != null) {
            fileContentBuilder.append(line)
          }

          val fileContent = fileContentBuilder.toString()
          navGridJsonPub.set(fileContent)
        }
      } catch (e: Exception) {
        DriverStation.reportError(
          "RemoteADStar failed to load navgrid. Pathfinding will not be functional.",
          false
        )
      }
    }
  }

  /**
   * Get if a new path has been calculated since the last time a path was retrieved
   *
   * @return True if a new path is available
   */
  override fun isNewPathAvailable(): Boolean {
    return newPathAvailable.get()
  }

  /**
   * Get the most recently calculated path
   *
   * @param constraints The path constraints to use when creating the path
   * @param goalEndState The goal end state to use when creating the path
   * @return The PathPlannerPath created from the points calculated by the pathfinder
   */
  override fun getCurrentPath(constraints: PathConstraints, goalEndState: GoalEndState): PathPlannerPath? {
    val pathPoints: List<PathPoint> = ArrayList(currentPath.get())

    newPathAvailable.set(false)

    if (pathPoints.size < 2) {
      return null
    }

    return PathPlannerPath.fromPathPoints(pathPoints, constraints, goalEndState)
  }

  /**
   * Set the start position to pathfind from
   *
   * @param startPosition Start position on the field. If this is within an obstacle it will be
   * moved to the nearest non-obstacle node.
   */
  override fun setStartPosition(startPosition: Translation2d) {
    startPosPub.set(doubleArrayOf(startPosition.x, startPosition.y))
  }

  /**
   * Set the goal position to pathfind to
   *
   * @param goalPosition Goal position on the field. f this is within an obstacle it will be moved
   * to the nearest non-obstacle node.
   */
  override fun setGoalPosition(goalPosition: Translation2d) {
    goalPosPub.set(doubleArrayOf(goalPosition.x, goalPosition.y))
  }

  /**
   * Set the dynamic obstacles that should be avoided while pathfinding.
   *
   * @param obs A List of Translation2d pairs representing obstacles. Each Translation2d represents
   * opposite corners of a bounding box.
   * @param currentRobotPos The current position of the robot. This is needed to change the start
   * position of the path to properly avoid obstacles
   */
  override fun setDynamicObstacles(
    obs: List<Pair<Translation2d, Translation2d>>,
    currentRobotPos: Translation2d
  ) {
    val obsArr = DoubleArray(((obs.size * 2) + 1) * 2)

    // First two doubles represent current robot pos
    obsArr[0] = currentRobotPos.x
    obsArr[1] = currentRobotPos.y

    var idx = 2
    for (box in obs) {
      obsArr[idx] = box.first.x
      obsArr[idx + 1] = box.first.y
      obsArr[idx + 2] = box.second.x
      obsArr[idx + 3] = box.second.y

      idx += 4
    }

    dynamicObsPub.set(obsArr)
  }
}
