package org.firstinspires.ftc.teamcode.dairy.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.dairy.subsystems.OuttakeClaw
import org.firstinspires.ftc.teamcode.dairy.util.Waiter
@OuttakeClaw.Attach

class OuttakePitchUp() : Action {
    private var init = false
    private lateinit var waiter:Waiter

    override fun run(p: TelemetryPacket): Boolean {
        if (!init) {
            waiter = Waiter()
            waiter.start(200)
            OuttakeClaw.pitch_pos = OuttakeClaw.pitch_up
            init = true
        }

        return waiter.isDone
    }

    companion object {
        fun outtakePitchUp(): Action {
            return OuttakePitchUp()
        }}
}