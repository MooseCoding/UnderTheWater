package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.dairy.control.FullController;

@TeleOp
@Config
public class Tuning extends LinearOpMode {
    private PIDFController controller;

    @Override
    public void runOpMode() throws InterruptedException {
        Lift l = new Lift(hardwareMap);
        Intake i = new Intake(hardwareMap);
        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            if(gamepad1.cross) {
                Intake.target = 500;
            }
            if(gamepad1.square) {
                i.pitchDown();
            }
            if(gamepad1.triangle) {
                i.clawOpen();
            }
            if(gamepad1.circle) {
                i.clawClose();
                i.pitchUp();
            }

            if(gamepad1.right_bumper) {
                Intake.target = 0;
            }

            if(gamepad1.dpad_up) {
                l.clawClose();
            }

            if(gamepad1.dpad_right) {
                i.clawPartial ();
            }

            if(gamepad1.dpad_down) {
                Lift.target = 4000;
            }

            if(gamepad1.dpad_left) {
                l.pitchDrop();
            }

            if(gamepad1.left_bumper) {
                l.clawOpen();
            }

            l.update();
            i.update();

            if(gamepad1.right_stick_button) {
                i.iM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if(gamepad1.left_stick_button) {
                l.oM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                l.oM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
            }

            telemetry.addData("om1 pos", l.oM1.getCurrentPosition());
            telemetry.addData("om2 pos", l.oM2.getCurrentPosition());
            telemetry.addData("i pos", i.iM.getCurrentPosition());
            telemetry.update();
        }
    }
}
