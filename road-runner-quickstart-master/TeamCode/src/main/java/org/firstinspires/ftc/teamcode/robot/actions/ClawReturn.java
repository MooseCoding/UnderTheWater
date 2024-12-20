package org.firstinspires.ftc.teamcode.robot.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.robot.Lift;

public class ClawReturn implements Action {
    private boolean init = false;
    private Lift claw;

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if(!init) {
            claw.pitchHome();
            claw.clawOpen();
            init = true;
        }

        return false;
    }

    public ClawReturn(Lift li) {
        claw = li;
    }

    public Action clawReturn(Lift li) {
        return new org.firstinspires.ftc.teamcode.dairy.actions.ClawReturn(li);
    }
}