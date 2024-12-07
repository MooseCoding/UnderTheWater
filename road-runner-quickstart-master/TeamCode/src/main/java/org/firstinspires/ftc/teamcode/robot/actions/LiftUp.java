package org.firstinspires.ftc.teamcode.robot.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.robot.Lift;

public class LiftUp implements Action {
    private boolean init = false;
    private Lift lift;
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if(!init) {
            lift.sample();
            init = true;
        }

        if(lift.oM1.getCurrentPosition() < 2800) {
            return true;
        }

        return false;
    }

    public LiftUp(Lift li) {
        lift = li;
    }

    public Action liftUp(Lift li) {
        return new LiftUp(li);
    }
}