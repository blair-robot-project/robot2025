package frc.team449.system.encoder

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import kotlin.math.pow

/** Intended use with REV throughbore encoder to calibrate on the low side of the absolute encoder reading wobble
 *    and will apply the offset to the relative quadrature output.  */
class QuadCalibration(
  subsystem: Subsystem,
  private val absolute: AbsoluteEncoder,
  private val encoder: QuadEncoder,
  private val numSamples: Int = 250,
  private val name: String = ""
) : Command() {

  init {
    addRequirements(subsystem)
  }

  private var samples = mutableListOf<Double>()

  override fun initialize() {
    println("**** Starting Encoder Calibration for $name ****")
  }

  override fun execute() {
    samples.add(absolute.position)
  }

  override fun isFinished(): Boolean {
    return samples.size >= numSamples
  }

  override fun end(interrupted: Boolean) {
    val angle = clusterAndComputeLowAverage(samples)
    encoder.resetPosition(angle)
    println("**** Finished Calibrating $name Quadrature reading, Low Side Avg of $angle ****")
  }

  // K-Means clustering implementation
  private fun kMeans(data: List<Double>, k: Int, maxIterations: Int = 1000): Map<Int, List<Double>> {
    require(k > 0) { "Number of clusters must be greater than 0." }
    require(data.isNotEmpty()) { "Data must not be empty." }

    // Initialize centroids randomly from the data points
    val centroids = data.shuffled().take(k).toMutableList()

    var clusters = mutableMapOf<Int, MutableList<Double>>()
    repeat(maxIterations) {
      // Clear the clusters for the current iteration
      clusters = mutableMapOf()

      // Assign each data point to the nearest centroid
      for (point in data) {
        val closestCentroidIndex = centroids.indices.minByOrNull { i ->
          (point - centroids[i]).pow(2)
        } ?: 0
        clusters.computeIfAbsent(closestCentroidIndex) { mutableListOf() }.add(point)
      }

      // Update centroids as the mean of the assigned points
      var changed = false
      for ((index, points) in clusters) {
        val newCentroid = points.average()
        if (newCentroid != centroids[index]) {
          centroids[index] = newCentroid
          changed = true
        }
      }

      // If centroids don't change, stop early
      if (!changed) return clusters
    }

    return clusters
  }

  // Function to separate data into "high" and "low" clusters and compute the average of the low cluster
  private fun clusterAndComputeLowAverage(data: List<Double>): Double {
    // Run K-Means clustering with 2 clusters
    val clusters = kMeans(data, k = 2)

    // Identify which cluster is "low" based on the average value of the clusters
    val lowClusterIndex = clusters.entries.minByOrNull { (_, points) -> points.average() }?.key ?: 0

    // Extract the low section and calculate the average
    val lowSection = clusters[lowClusterIndex] ?: emptyList()
    val lowAverage = lowSection.average()

    return lowAverage
  }
}
