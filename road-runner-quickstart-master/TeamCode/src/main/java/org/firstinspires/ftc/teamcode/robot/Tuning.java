package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.dairy.control.FullController;

@TeleOp
@Config
@Disabled
public class Tuning extends LinearOpMode {
    private PIDFController controller;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Lift l = new Lift(hardwareMap);
        Intake i = new Intake(hardwareMap);
        Hang h = new Hang(hardwareMap);

        Lift.pidfused = false;
        Intake.pidused = false;
        Hang.pidfused = false;

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            l.update();
            i.update();

            if(gamepad1.right_stick_button) {
                i.iM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if(gamepad1.left_stick_button) {
                l.oM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                l.oM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if(gamepad1.dpad_up) {
                Hang.hM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }


            telemetry.addData("om1 pos", l.oM1.getCurrentPosition());
            telemetry.addData("om2 pos", l.oM2.getCurrentPosition());
            telemetry.addData("i pos", i.iM.getCurrentPosition());
            telemetry.addData("hm", Hang.hM.getCurrentPosition());
            telemetry.update();
        }
    }
}
