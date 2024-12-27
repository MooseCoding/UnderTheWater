package org.firstinspires.ftc.teamcode.dairy.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.dairy.subsystems.IntakeClaw
import org.firstinspires.ftc.teamcode.dairy.util.Waiter

class IntakeClawPartial() : Action {
    private var init = false
    private lateinit var waiter:Waiter

    override fun run(p: TelemetryPacket): Boolean {
        if (!init) {
            IntakeClaw.claw_pos = IntakeClaw.claw_partial
            waiter.start(200)
            init = true
        }

        return waiter.isDone
    }

    companion object {
        fun intakeClawPartial(): Action {
            return IntakeClawPartial()
        }
    }
}