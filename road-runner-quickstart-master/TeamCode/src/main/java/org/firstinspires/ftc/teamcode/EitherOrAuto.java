package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.roadrunner.PoseStorage.currentPose;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Lift;


@Photon
@Autonomous
public class EitherOrAuto extends LinearOpMode {
    private enum AUTO {
        RED,
        BLUE,
    }

    private AUTO color = AUTO.RED;


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive d = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Pose2d init_pos = new Pose2d(-12, -61, Math.PI);

        while(!isStopRequested()) {
            Lift lift = new Lift(hardwareMap);
            Intake in = new Intake(hardwareMap);

            TrajectoryActionBuilder dropOff = d.actionBuilder(init_pos)
                    .lineToX(-50);

            TrajectoryActionBuilder firstSamplePickUp = d.actionBuilder(d.pose)
                    .splineTo(new Vector2d(-48, -37), Math.PI/2);

            TrajectoryActionBuilder firstSampleDropOff = d.actionBuilder(d.pose)
                    .turn(Math.PI/2)
                    .strafeTo(new Vector2d(-52, -60));

            TrajectoryActionBuilder secondSamplePickUp = d.actionBuilder(d.pose)
                    .strafeTo(new Vector2d(-58, -35))
                    .turn(-Math.PI/2);

            TrajectoryActionBuilder secondSampleDropOff = d.actionBuilder(d.pose)
                    .strafeTo(new Vector2d(-52, -60))
                    .turn(Math.PI/2);

            TrajectoryActionBuilder thirdSamplePickUp = d.actionBuilder(d.pose)
                    .strafeTo(new Vector2d(-55, -28));

            TrajectoryActionBuilder thirdSampleDropOff = d.actionBuilder(d.pose)
                    .strafeTo(new Vector2d(-52, -60));

            TrajectoryActionBuilder ascent = d.actionBuilder(d.pose)
                    .strafeTo(new Vector2d(-52, -40))

                    .splineTo(new Vector2d(-24, -12), 0)
                    .turn(-Math.PI/2);


            waitForStart();
/*
            switch (color) {
                case RED: {
                    Actions.runBlocking(
                            new SequentialAction(
                                    dropOff,

                            )
                    );
                }
                case BLUE: {

                }
            }*/

            Actions.runBlocking(
                    new SequentialAction(

                    )
            );

            currentPose = d.pose;
        }
    }
}