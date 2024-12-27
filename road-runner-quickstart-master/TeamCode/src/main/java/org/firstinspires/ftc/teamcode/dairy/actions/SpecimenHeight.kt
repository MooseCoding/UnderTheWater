package org.firstinspires.ftc.teamcode.dairy.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift
import org.firstinspires.ftc.teamcode.dairy.subsystems.OuttakeClaw

@Lift.Attach
class SpecimenHeight() : Action {
    private var init = false

    override fun run(p: TelemetryPacket): Boolean {
        if (!init) {
            Lift.pidfused = true
            Lift.target = 2000.0
            init = true
        }

        Lift.update()

        if(Lift.outtake1!!.currentPosition > 1950) {
            return true
        }

        return false
    }

    companion object {
        fun specimenHeight(): Action {
            return SpecimenHeight()
        }
    }
}