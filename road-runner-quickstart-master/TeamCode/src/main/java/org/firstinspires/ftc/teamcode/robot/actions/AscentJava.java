package org.firstinspires.ftc.teamcode.robot.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.robot.Lift;

public class Ascent implements Action {
    private boolean init = false;
    private Lift claw;

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if(!init) {
            Lift.target = 1000;
            init = true;
        }

        return claw.oM1.getCurrentPosition() < 800;
    }

    public Ascent(Lift li) {
        claw = li;
    }

    public Action ascent(Lift li) {
        return new org.firstinspires.ftc.teamcode.dairy.actions.Ascent(li);
    }
}