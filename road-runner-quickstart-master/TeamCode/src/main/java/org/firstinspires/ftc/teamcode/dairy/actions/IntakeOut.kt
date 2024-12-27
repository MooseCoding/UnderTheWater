package org.firstinspires.ftc.teamcode.dairy.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import org.firstinspires.ftc.teamcode.dairy.subsystems.Intake
import org.firstinspires.ftc.teamcode.dairy.util.Waiter

class IntakeOut() : Action {
    private var init = false
    private lateinit var waiter: Waiter

    override fun run(p: TelemetryPacket): Boolean {
        if (!init) {
            Intake.target = 300.0
            Intake.pidused = true
            init = true
        }

        return Intake.intake!!.currentPosition >= 280
    }

    companion object {
        fun intakeOut(): Action {
            return IntakeOut()
        }}
}