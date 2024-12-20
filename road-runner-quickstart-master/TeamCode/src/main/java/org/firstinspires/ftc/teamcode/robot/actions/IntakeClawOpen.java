package org.firstinspires.ftc.teamcode.robot.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.robot.Intake;

public class IntakeClawOpen implements Action {
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

    public IntakeClawOpen(Intake li) {
        i = li;
    }

    public Action intakeClawOpen(Intake li) {
        return new org.firstinspires.ftc.teamcode.dairy.actions.IntakeClawOpen(li);
    }
}