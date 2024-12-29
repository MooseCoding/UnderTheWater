package org.firstinspires.ftc.teamcode.dairy.tuning;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.old.teamcode.MecanumDrive;

@Autonomous
public class MATune extends OpMode {
    private MecanumDrive d;
    private Pose2d init_pos = new Pose2d(12.0, 61.0,-Math.PI/2);

    private Action dropOff;

    @Override
    public void init() {
        d = new MecanumDrive(hardwareMap, init_pos);

        dropOff = d.actionBuilder(init_pos)
                .strafeTo(new Vector2d(12,38))
                .build();
    }

    @Override
    public void init_loop() {
        Actions.runBlocking(
                dropOff
        );
    }

    @Override
    public void loop() {
        d.updatePoseEstimate();
        telemetry.addData("d pose", d.pose);
        telemetry.update();
    }
}
