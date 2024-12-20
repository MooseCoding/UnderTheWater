package org.firstinspires.ftc.teamcode.dairy.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.dairy.subsystems.OuttakeClaw
import org.firstinspires.ftc.teamcode.dairy.util.Waiter

class OuttakeClawOpen() : Action {
    private var init = false
    private lateinit var waiter: Waiter

    override fun run(p: TelemetryPacket): Boolean {
        if (!init) {
            OuttakeClaw.INSTANCE.clawOpen()
            waiter = Waiter()
            waiter.start(200)
            init = true
        }

        return !waiter.isDone
    }

    companion object {
    fun outtakeClawOpen(): Action {
        return OuttakeClawOpen()
    }}
}