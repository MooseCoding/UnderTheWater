package org.firstinspires.ftc.teamcode.dairy.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift
import org.firstinspires.ftc.teamcode.dairy.subsystems.OuttakeClaw

@Lift.Attach
class SpecimenHeightI() : Action {
    private var init = false

    override fun run(p: TelemetryPacket): Boolean {
        if (!init) {
            Lift.pidfused = true
            Lift.target = 1600.0
            init = true
        }

        p.put("oM1Pos",Lift.outtake1!!.currentPosition)
        p.put("pidfUsed", Lift.pidfused)

        Lift.update()

        if(Lift.outtake1!!.currentPosition > 1550) {
            return true
        }

        return false
    }

    companion object {
        fun specimenHeightI(): Action {
            return SpecimenHeightI()
        }
    }
}