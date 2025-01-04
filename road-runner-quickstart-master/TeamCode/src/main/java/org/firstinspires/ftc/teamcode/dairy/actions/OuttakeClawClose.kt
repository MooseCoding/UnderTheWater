package org.firstinspires.ftc.teamcode.dairy.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.dairy.subsystems.OuttakeClaw
import org.firstinspires.ftc.teamcode.dairy.util.Waiter

@OuttakeClaw.Attach
class OuttakeClawClose() : Action {
    private var init = false
    private lateinit var waiter:Waiter

    override fun run(p: TelemetryPacket): Boolean {
        if (!init) {
            waiter = Waiter()
            waiter.start(200)
            OuttakeClaw.claw_pos = OuttakeClaw.claw_close
            init = true
        }

        if(waiter.isDone) {
            return true
        }

        return false
    }

    companion object {
        fun outtakeClawClose(): Action {
            return OuttakeClawClose()
        }
    }
}