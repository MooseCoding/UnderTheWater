package org.firstinspires.ftc.teamcode.dairy.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.dairy.subsystems.IntakeClaw
import org.firstinspires.ftc.teamcode.dairy.util.Waiter

class IntakeClawOpen() : Action {
    private var init = false
    private lateinit var waiter:Waiter

    override fun run(p: TelemetryPacket): Boolean {
        if (!init) {
            waiter = Waiter()
            waiter.start(200)
            IntakeClaw.claw_pos = IntakeClaw.claw_open
            init = true
        }

        return waiter.isDone
    }

    companion object {
        fun intakeClawOpen(): Action {
            return IntakeClawOpen()
        }
    }
}