package org.firstinspires.ftc.teamcode.dairy.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.dairy.subsystems.IntakeClaw
import org.firstinspires.ftc.teamcode.dairy.util.Waiter

class IntakePitchDown() : Action {
    private var init = false
    private lateinit var waiter:Waiter

    override fun run(p: TelemetryPacket): Boolean {
        if (!init) {
            waiter = Waiter()
            waiter.start(400)
            IntakeClaw.pitch_pos = IntakeClaw.pitch_down
            init = true
        }

        return waiter.isDone
    }

    companion object {
    fun intakePitchDown(): Action {
        return IntakePitchDown()
    }}
}