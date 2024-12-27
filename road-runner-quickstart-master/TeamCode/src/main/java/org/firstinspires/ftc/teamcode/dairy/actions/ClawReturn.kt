package org.firstinspires.ftc.teamcode.dairy.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.dairy.subsystems.OuttakeClaw
import org.firstinspires.ftc.teamcode.dairy.util.Waiter

class ClawReturn() : Action {
    private var init = false
    private var i = 0
    private lateinit var waiter:Waiter

    override fun run(p: TelemetryPacket): Boolean {
        if (!init) {
            waiter = Waiter()
            waiter.start(200)
            OuttakeClaw.pitch_pos = OuttakeClaw.pitch_down
            init = true
        }

        if(waiter.isDone && i == 0) {
            OuttakeClaw.claw_pos = OuttakeClaw.claw_open
            i = 1
        }

        return i == i
    }

    companion object {
    fun clawReturn(): Action {
        return ClawReturn()
    }}
}