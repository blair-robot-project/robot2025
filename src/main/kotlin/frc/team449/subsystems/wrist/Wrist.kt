package frc.team449.subsystems.wrist

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.createKraken

class Wrist(private val motor: TalonFX) : SubsystemBase() {

    companion object {
        fun createWrist(): Wrist {
            val motor: TalonFX = createKraken(WristConstants.MOTOR_ID, false)

            return Wrist(motor)
        }
    }
}
