package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import edu.wpi.first.wpilibj2.command.Commands
import frc.team449.Robot

class Routines(
  robot: Robot
) {
  val autoFactory = AutoFactory(
    robot.poseSubsystem::pose,
    robot.poseSubsystem::resetOdometry,
    robot.drive::followTrajectory,
    true,
    robot.drive
  )

  // do nothing
  fun doNothing(): AutoRoutine {
    val nothing: AutoRoutine = autoFactory.newRoutine("Nothing")
    return nothing
  }

  // right taxi
  fun rightTaxi(): AutoRoutine {
    val rTaxi: AutoRoutine = autoFactory.newRoutine("Right Taxi")
    val rTaxiTrajectory: AutoTrajectory = rTaxi.trajectory("rightTaxi")
    rTaxi.active().onTrue(
      Commands.sequence(
        rTaxiTrajectory.resetOdometry(),
        rTaxiTrajectory.cmd()
      )
    )
    return rTaxi
  }

  // left taxi
  fun leftTaxi(): AutoRoutine {
    val lTaxi: AutoRoutine = autoFactory.newRoutine("Left Taxi")
    val lTaxiTrajectory: AutoTrajectory = lTaxi.trajectory("leftTaxi")
    lTaxi.active().onTrue(
      Commands.sequence(
        lTaxiTrajectory.resetOdometry(),
        lTaxiTrajectory.cmd()
      )
    )
    return lTaxi
  }

  // coral at l1 one on reef E
  fun reefE(): AutoRoutine {
    val E: AutoRoutine = autoFactory.newRoutine("L1 reef E")
    val reefETrajectory: AutoTrajectory = E.trajectory("right_E")
    E.active().onTrue(
      Commands.sequence(
        reefETrajectory.resetOdometry(),
        reefETrajectory.cmd()
      )
    )
    return E
  }

  // coral at l1 one on reef F
  fun reefF(): AutoRoutine {
    val F: AutoRoutine = autoFactory.newRoutine("L1 reef F")
    val reefFTrajectory: AutoTrajectory = F.trajectory("left_F")
    F.active().onTrue(
      Commands.sequence(
        reefFTrajectory.resetOdometry(),
        reefFTrajectory.cmd()
      )
    )
    return F
  }

  // coral at l1 one on reef G
  fun reefG(): AutoRoutine {
    val g: AutoRoutine = autoFactory.newRoutine("L1 reef G")
    val reefGTrajectory: AutoTrajectory = g.trajectory("middle_G")
    g.active().onTrue(
      Commands.sequence(
        reefGTrajectory.resetOdometry(),
        reefGTrajectory.cmd()
      )
    )
    return g
  }

  // coral at l1 one on reef J
  fun reefJ(): AutoRoutine {
    val J: AutoRoutine = autoFactory.newRoutine("L1 reef J")
    val reefJTrajectory: AutoTrajectory = J.trajectory("left_J")
    J.active().onTrue(
      Commands.sequence(
        reefJTrajectory.resetOdometry(),
        reefJTrajectory.cmd()
      )
    )
    return J
  }

  // placing L4 at reef E then l1 at reef D
  fun reefED(): AutoRoutine {
    val E_D: AutoRoutine = autoFactory.newRoutine("L4 reef E and L1 reef D")
    val reefETrajectory: AutoTrajectory = E_D.trajectory("right_E")
    val reefEtoStationTrajectory: AutoTrajectory = E_D.trajectory("E_rightStation")
    val stationToDTrajectory: AutoTrajectory = E_D.trajectory("rightStation_D")
    E_D.active().onTrue(
      Commands.sequence(
        Commands.sequence(
          reefETrajectory.resetOdometry(),
          reefETrajectory.cmd(),
          Commands.waitSeconds(0.7)
        ),
        Commands.sequence(
          reefEtoStationTrajectory.cmd(),
          Commands.waitSeconds(0.9)
        ),
        Commands.sequence(
          stationToDTrajectory.cmd()
        )
      )
    )
    return E_D
  }

  // placing L4 at reef E then l1 at reef L
  fun reefJL(): AutoRoutine {
    val J_L: AutoRoutine = autoFactory.newRoutine("L4 reef J and L1 reef L")
    val reefJTrajectory: AutoTrajectory = J_L.trajectory("left_J")
    val reefJtoStationTrajectory: AutoTrajectory = J_L.trajectory("J_leftStation")
    val stationToLTrajectory: AutoTrajectory = J_L.trajectory("leftStation_L")
    J_L.active().onTrue(
      Commands.sequence(
        Commands.sequence(
          reefJTrajectory.resetOdometry(),
          reefJTrajectory.cmd(),
          Commands.waitSeconds(0.7)
        ),
        Commands.sequence(
          reefJtoStationTrajectory.cmd(),
          Commands.waitSeconds(0.9)
        ),
        Commands.sequence(
          stationToLTrajectory.cmd()
        )
      )
    )
    return J_L
  }

  fun reefJK(): AutoRoutine {
    val J_K: AutoRoutine = autoFactory.newRoutine("L4 reef J and L1 reef K")
    val reefJTrajectory: AutoTrajectory = J_K.trajectory("left_J")
    val reefJtoStationTrajectory: AutoTrajectory = J_K.trajectory("J_leftStation")
    val stationToKTrajectory: AutoTrajectory = J_K.trajectory("leftStation_K")
    J_K.active().onTrue(
      Commands.sequence(
        Commands.sequence(
          reefJTrajectory.resetOdometry(),
          reefJTrajectory.cmd(),
          Commands.waitSeconds(0.7)
        ),
        Commands.sequence(
          reefJtoStationTrajectory.cmd(),
          Commands.waitSeconds(0.9)
        ),
        Commands.sequence(
          stationToKTrajectory.cmd()
        )
      )
    )
    return J_K
  }

// autoChooser that will be displayed on dashboard
  fun addOptions(autoChooser: AutoChooser) {
    autoChooser.addRoutine("Nothing", this::doNothing)
    autoChooser.addRoutine("RightTaxi", this::rightTaxi)
    autoChooser.addRoutine("LeftTaxi", this::leftTaxi)
    autoChooser.addRoutine("E", this::reefE)
    autoChooser.addRoutine("F", this::reefF)
    autoChooser.addRoutine("G", this::reefG)
    autoChooser.addRoutine("J", this::reefJ)
    autoChooser.addRoutine("E & D ", this::reefED)
    autoChooser.addRoutine("J & L", this::reefJL)
    autoChooser.addRoutine("J & K", this::reefJK)
  }
}
