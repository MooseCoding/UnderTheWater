package org.firstinspires.ftc.teamcode.dairy.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.dairy.subsystems.IntakeClaw

class IntakeClawOpen() : Action {
    private var init = false

    override fun run(p: TelemetryPacket): Boolean {
        if (!init) {
            IntakeClaw.INSTANCE.openClaw()
            init = true
        }

        return false
    }

    companion object {
        fun intakeClawOpen(): Action {
            return IntakeClawOpen()
        }
    }
}