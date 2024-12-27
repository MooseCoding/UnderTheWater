package org.firstinspires.ftc.teamcode.dairy.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift
import org.firstinspires.ftc.teamcode.dairy.subsystems.OuttakeClaw

@Lift.Attach
class SampleHeight() : Action {
    private var init = false

    override fun run(p: TelemetryPacket): Boolean {
        if (!init) {
            Lift.pidfused = true
            Lift.target = 3900.0
            init = true
        }

        Lift.update()

        return Lift.outtake1!!.currentPosition >= 3800
    }

    companion object {
        fun sampleHeight(): Action {
            return SampleHeight()
        }
    }
}