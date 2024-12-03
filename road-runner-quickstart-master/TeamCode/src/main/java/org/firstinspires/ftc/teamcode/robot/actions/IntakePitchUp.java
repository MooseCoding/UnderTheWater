package org.firstinspires.ftc.teamcode.robot.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.robot.Intake;

public class IntakePitchUp implements Action {
    private boolean init = false;
    private Intake i;

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if(!init) {
            i.pitchUp();
            init = true;
        }

        return false;
    }

    public IntakePitchUp(Intake li) {
        i = li;
    }

    public Action intakePitchUp(Intake li) {
        return new IntakePitchUp(li);
    }
}