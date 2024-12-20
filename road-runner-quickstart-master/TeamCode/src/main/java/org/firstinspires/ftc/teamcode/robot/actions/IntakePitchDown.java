package org.firstinspires.ftc.teamcode.robot.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.robot.Intake;

public class IntakePitchDown implements Action {
    private boolean init = false;
    private Intake i;

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if(!init) {
            i.pitchDown();
            init = true;
        }

        return false;
    }

    public IntakePitchDown(Intake li) {
        i = li;
    }

    public Action intakePitchDown(Intake li) {
        return new org.firstinspires.ftc.teamcode.dairy.actions.IntakePitchDown(li);
    }
}