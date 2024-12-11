package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.roadrunner.teamcode.PoseStorage.currentPose;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Lift;


@Photon
@Autonomous
public class BlueAuto extends LinearOpMode {

    private Lift lift;
    private Intake in;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive d = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Pose2d init_pos = new Pose2d(12, 61, 0);

        while(!isStopRequested()) {
             lift = new Lift(hardwareMap);
             in = new Intake(hardwareMap);

            TrajectoryActionBuilder dropOff = d.actionBuilder(init_pos)
                    .lineToY(36);

            TrajectoryActionBuilder dropOffToAscent = d.actionBuilder(d.pose)
                    .lineToY(40)
                    .strafeTo(new Vector2d(35, 40))
                    .strafeTo(new Vector2d(35, 0))
                    .turn(-Math.PI/2)
                    .lineToX(24);

            TrajectoryActionBuilder firstSamplePickUp = d.actionBuilder(d.pose)
                    .strafeTo(new Vector2d(48, 37));

            TrajectoryActionBuilder firstSampleDropOff = d.actionBuilder(d.pose)
                    .turn(Math.PI/2)
                    .strafeTo(new Vector2d(52, 60));

            TrajectoryActionBuilder firstSampleToAscent = d.actionBuilder(d.pose)
                    .strafeTo(new Vector2d(37,0))
                    .turn(Math.PI/-1)
                    .strafeTo(new Vector2d(24, 0));


            TrajectoryActionBuilder secondSamplePickUp = d.actionBuilder(d.pose)
                    .strafeTo(new Vector2d(58, 35))
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

            Actions.runBlocking(
                    Lift.clawGrab()
            );

            waitForStart();

            Actions.runBlocking(
                    new SequentialAction(
                            new Lift.SpecimenHeight(),
                            new Lift.PitchUp(),
                            dropOff.build(),
                            new Lift.SpecimenDown(),
                            new Lift.ClawDrop(),
                            dropOffToAscent.build()
                    )
            );

            currentPose = d.pose;
        }
    }
}