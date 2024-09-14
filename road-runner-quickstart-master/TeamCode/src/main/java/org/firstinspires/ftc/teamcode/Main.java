package org.firstinspires.ftc.teamcode;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Photon
public class Main extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStopRequested()) {
            DcMotorEx fL = hardwareMap.dcMotor.get("frontLeftMotor");
            DcMotorEx bL = hardwareMap.dcMotor.get("backLeftMotor");
            DcMotorEx fR = hardwareMap.dcMotor.get("frontRightMotor");
            DcMotorEx bR = hardwareMap.dcMotor.get("backRightMotor");

            // Set Motors in Reverse

            waitForStart();

            if (isStopRequested()) {return;}

            while(opModeIsActive()) {
                double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double rx = gamepad1.right_stick_x;
                double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing

                // This ensures all the powers maintain the same ratio,
                // Denominator is the largest motor power (absolute value) or 1
                // but only if at least one is out of the range [-1, 1]
                double d = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1); // Denominator 
                double fLP = (y + x + rx) / d; // Front left motor power
                double bLP = (y - x + rx) / d; // Back left motor power
                double fRP = (y - x - rx) / d; // Front right motor power
                double bRP = (y + x - rx) / d; // Back right motor power

                fL.setPower(fLP);
                bL.setPower(bLP);
                fR.setPower(fRP);
                bR.setPower(bRP);
            }
        }
    }
}
