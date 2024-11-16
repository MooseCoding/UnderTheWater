package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.vision.Color;
import org.firstinspires.ftc.teamcode.vision.Sample;
import org.firstinspires.ftc.teamcode.vision.robotPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;



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
        while(!isStopRequested()) {
            Lift lift = new Lift(hardwareMap);


            waitForStart();

            switch (color) {
                case RED: {
                    lift.holdSample();
                    lift.DriveLift(0.8, 3000);
                    lift.dropSample();
                    double t = getRuntime();
                    while(getRuntime() < t + 0.4) {}
                    lift.returnClaw();
                    lift.DriveLift(0.3, 0);
                }
                case BLUE: {

                }
            }
        }
    }
}