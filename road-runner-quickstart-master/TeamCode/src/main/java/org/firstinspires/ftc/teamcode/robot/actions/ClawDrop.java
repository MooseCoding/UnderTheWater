package org.firstinspires.ftc.teamcode.robot.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.robot.Lift;

public class ClawDrop implements Action {
    private boolean init = false;
    private Lift claw;

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if(!init) {
            claw.pitchDrop();
            claw.clawOpen();
            init = true;
        }

        return false;
    }

    public ClawDrop(Lift li) {
        claw = li;
    }

    public Action clawDrop(Lift li) {
        return new ClawDrop(li);
    }
}