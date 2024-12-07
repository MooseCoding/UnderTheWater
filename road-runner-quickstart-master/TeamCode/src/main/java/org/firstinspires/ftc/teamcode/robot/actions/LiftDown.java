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
            lift.home();
            init = true;
        }

       if(lift.oM1.getCurrentPosition() > 200) {
           lift.update();
           return true;
       }

        return false;
    }

    public LiftDown(Lift li) {
        lift = li;
    }

    public Action liftDown(Lift li) {
        return new LiftDown(li);
    }
}