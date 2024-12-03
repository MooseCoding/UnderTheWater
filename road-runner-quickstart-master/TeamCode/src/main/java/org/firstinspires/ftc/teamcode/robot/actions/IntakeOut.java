package org.firstinspires.ftc.teamcode.robot.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.robot.Intake;

public class IntakeOut implements Action {
    private boolean init = false;
    private Intake i;

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if(!init) {
            Intake.target = 200;
            init = true;
        }

        return i.iM.getCurrentPosition() < 150;
    }

    public IntakeOut(Intake li) {
        i = li;
    }

    public Action intakeOut(Intake li) {
        return new IntakeOut(li);
    }
}