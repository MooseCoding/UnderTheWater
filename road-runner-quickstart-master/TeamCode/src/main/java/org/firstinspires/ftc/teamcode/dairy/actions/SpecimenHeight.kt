package org.firstinspires.ftc.teamcode.dairy.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift
import org.firstinspires.ftc.teamcode.dairy.subsystems.OuttakeClaw

class SpecimenHeight() : Action {
    private var init = false

    override fun run(p: TelemetryPacket): Boolean {
        if (!init) {
            Lift.target = 2000.0
            init = true
        }

        Lift.update()

        return Lift.outtake1!!.currentPosition <= 1900
    }

    companion object {
        fun outtakeClawClose(): Action {
            return OuttakeClawClose()
        }
    }
}