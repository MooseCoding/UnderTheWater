package org.firstinspires.ftc.teamcode.dairy.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.dairy.subsystems.Lift

class Ascent() : Action {
    private var init = false

    override fun run(p: TelemetryPacket): Boolean {
        if (!init) {
            Lift.target = 1000.0
            init = true
        }

        return Lift.outtake1!!.currentPosition > 900
    }

    companion object {
    fun ascent(): Action {
        return Ascent()
    }}
}