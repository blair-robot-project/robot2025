package frc.team449

import edu.wpi.first.wpilibj.RobotBase

object Main {
  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the argument type.
   */
  @JvmStatic
  fun main(args: Array<String>) {
    RobotBase.startRobot { RobotLoop() }
  }
}