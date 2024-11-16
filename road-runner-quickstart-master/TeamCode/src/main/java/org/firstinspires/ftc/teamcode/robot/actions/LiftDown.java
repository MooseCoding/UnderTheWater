package org.firstinspires.ftc.teamcode.robot.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.robot.Lift;

public class LiftDown implements Action {
    private boolean init = false;
    private Lift lift;
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if(!init) {
            init = true;
        }

        lift.DriveLift(0.4, 0);

        if(lift.good2(0)) {
            return false;
        }
        else {
            return true;
        }
    }

    public LiftDown(Lift li) {
        lift = li;
    }

    public Action liftDown(Lift li) {
        return new LiftDown(li);
    }
}