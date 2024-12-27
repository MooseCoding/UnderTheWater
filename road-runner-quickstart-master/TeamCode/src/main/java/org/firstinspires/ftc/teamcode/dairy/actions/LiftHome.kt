package org.firstinspires.ftc.teamcode.dairy.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift
import org.firstinspires.ftc.teamcode.dairy.subsystems.OuttakeClaw

class LiftHome() : Action {
    private var init = false

    override fun run(p: TelemetryPacket): Boolean {
        if (!init) {
            Lift.pidfused = true
            Lift.target = 0.0
            init = true
        }

        Lift.update()

        if(Lift.outtake1!!.currentPosition >= 40) {
            Lift.pidfused = false
        }

        if(Lift.outtake1!!.currentPosition < 50) {
            return false
        }

        return true
    }

    companion object {
        fun liftHome(): Action {
            return SpecimenHeight()
        }
    }
}