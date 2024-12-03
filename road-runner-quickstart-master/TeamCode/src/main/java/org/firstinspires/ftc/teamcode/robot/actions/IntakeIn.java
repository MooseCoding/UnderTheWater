package org.firstinspires.ftc.teamcode.robot.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.robot.Intake;

public class IntakeIn implements Action {
    private boolean init = false;
    private Intake i;

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if(!init) {
            Intake.target = 0;
            init = true;
        }

        return i.iM.getCurrentPosition() > 40;
    }

    public IntakeIn(Intake li) {
        i = li;
    }

    public Action intakeIn(Intake li) {
        return new IntakeIn(li);
    }
}