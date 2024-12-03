package org.firstinspires.ftc.teamcode.robot.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.robot.Intake;

public class IntakeClawClose implements Action {
    private boolean init = false;
    private Intake i;

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if(!init) {
            i.clawOpen();
            init = true;
        }

        return false;
    }

    public IntakeClawClose(Intake li) {
        i = li;
    }

    public Action intakeClawClose(Intake li) {
        return new IntakeClawClose(li);
    }
}