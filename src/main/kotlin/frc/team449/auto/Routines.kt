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

  //do nothing
  fun doNothing():AutoRoutine{
    val nothing: AutoRoutine =autoFactory.newRoutine("Nothing")
    return nothing
  }

  //right taxi
  fun rightTaxi():AutoRoutine{
    val rTaxi: AutoRoutine = autoFactory.newRoutine("Right Taxi")
    val rTaxiTrajectory:AutoTrajectory = rTaxi.trajectory("rightTaxi")
    rTaxi.active().onTrue(
      Commands.sequence(
        rTaxiTrajectory.resetOdometry(),
        rTaxiTrajectory.cmd()
      )
    )
    return rTaxi
  }

  //left taxi
  fun leftTaxi():AutoRoutine{
    val lTaxi: AutoRoutine = autoFactory.newRoutine("Left Taxi")
    val lTaxiTrajectory:AutoTrajectory = lTaxi.trajectory("leftTaxi")
    lTaxi.active().onTrue(
      Commands.sequence(
        lTaxiTrajectory.resetOdometry(),
        lTaxiTrajectory.cmd()
      )
    )
    return lTaxi
  }

  //coral at l1 one on reef E
  fun reefEL1(): AutoRoutine {
    val l1E: AutoRoutine = autoFactory.newRoutine("L1 reef E")
    val reefETrajectory: AutoTrajectory = l1E.trajectory("right_E")
    l1E.active().onTrue(
      Commands.sequence(
        reefETrajectory.resetOdometry(),
        reefETrajectory.cmd()
      )
    )
    return l1E
  }

  //coral at l1 one on reef F
  fun reefFL1(): AutoRoutine {
    val l1F: AutoRoutine = autoFactory.newRoutine("L1 reef F")
    val reefFTrajectory: AutoTrajectory = l1F.trajectory("left_F")
    l1F.active().onTrue(
      Commands.sequence(
        reefFTrajectory.resetOdometry(),
        reefFTrajectory.cmd()
      )
    )
    return l1F
  }


  //coral at l1 one on reef G
  fun reefGL1(): AutoRoutine {
    val l1G: AutoRoutine = autoFactory.newRoutine("L1 reef G")
    val reefGTrajectory: AutoTrajectory = l1G.trajectory("middle_G")
    l1G.active().onTrue(
      Commands.sequence(
        reefGTrajectory.resetOdometry(),
        reefGTrajectory.cmd()
      )
    )
    return l1G
  }

  //coral at l1 one on reef J
  fun reefJL1(): AutoRoutine {
    val l1J: AutoRoutine = autoFactory.newRoutine("L1 reef J")
    val reefJTrajectory: AutoTrajectory = l1J.trajectory("left_J")
    l1J.active().onTrue(
      Commands.sequence(
        reefJTrajectory.resetOdometry(),
        reefJTrajectory.cmd()
      )
    )
    return l1J
  }


  //placing L4 at reef E then l1 at reef D
  fun reefEl4Dl1(): AutoRoutine {
    val l4E_l1D: AutoRoutine = autoFactory.newRoutine("L4 reef E and L1 reef D")
    val reefETrajectory: AutoTrajectory = l4E_l1D.trajectory("right_E")
    val reefEtoStationTrajectory: AutoTrajectory = l4E_l1D.trajectory("E_rightStation")
    val stationToDTrajectory: AutoTrajectory = l4E_l1D.trajectory("rightStation_D")
    l4E_l1D.active().onTrue(
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
    return l4E_l1D
  }



  //placing L4 at reef E then l1 at reef D
  fun reefl4Jl1L(): AutoRoutine {
    val l4J_l1L: AutoRoutine = autoFactory.newRoutine("L4 reef J and L1 reef D")
    val reefJTrajectory: AutoTrajectory = l4J_l1L.trajectory("left_J")
    val reefJtoStationTrajectory: AutoTrajectory = l4J_l1L.trajectory("J_leftStation")
    val stationToDTrajectory: AutoTrajectory = l4J_l1L.trajectory("leftStation_L")
    l4J_l1L.active().onTrue(
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
          stationToDTrajectory.cmd()
        )
      )
    )
    return l4J_l1L
  }

//autoChooser that will be displayed on dashboard
  fun addOptions(autoChooser: AutoChooser) {
    autoChooser.addRoutine("Nothing",this::doNothing)
    autoChooser.addRoutine("RightTaxi",this::rightTaxi)
    autoChooser.addRoutine("LeftTaxi",this::leftTaxi)
    autoChooser.addRoutine("L1E", this::reefEL1)
    autoChooser.addRoutine("L1F", this::reefFL1)
    autoChooser.addRoutine("L1G",this::reefGL1)
    autoChooser.addRoutine("L1J",this::reefJL1)
    autoChooser.addRoutine("l4E & L1D ", this::reefEl4Dl1)
    autoChooser.addRoutine("l1J & l4L",this::reefl4Jl1L)
  }
}
