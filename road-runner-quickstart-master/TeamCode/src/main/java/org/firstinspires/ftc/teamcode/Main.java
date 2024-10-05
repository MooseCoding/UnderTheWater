package org.firstinspires.ftc.teamcode;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Photon
@TeleOp
public class Main extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStopRequested()) {
            DcMotorEx fL = (DcMotorEx) hardwareMap.dcMotor.get("frontLeftMotor");
            DcMotorEx bL = (DcMotorEx) hardwareMap.dcMotor.get("backLeftMotor");
            DcMotorEx fR = (DcMotorEx) hardwareMap.dcMotor.get("frontRightMotor");
            DcMotorEx bR = (DcMotorEx) hardwareMap.dcMotor.get("backRightMotor");

            bR.setDirection(DcMotorSimple.Direction.REVERSE);
            fR.setDirection(DcMotorSimple.Direction.REVERSE);

            waitForStart();

            if (isStopRequested()) {
                return;
            }

            while (opModeIsActive()) {
                double max;

                double axial = -gamepad1.left_stick_y;
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;

                double leftFrontPower = (axial + lateral) + yaw;
                double rightFrontPower = (axial - lateral) - yaw;
                double leftBackPower = (axial - lateral) + yaw;
                double rightBackPower = (axial + lateral) - yaw;


                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }

                fL.setPower(leftFrontPower);
                bL.setPower(leftBackPower);
                bR.setPower(rightBackPower);
                fR.setPower(rightFrontPower);

                telemetry.addData("fLP",leftFrontPower);
                telemetry.addData("fRP", rightFrontPower);
                telemetry.addData("bLP", leftBackPower);
                telemetry.addData("bRP", rightBackPower);
                telemetry.update();
            }
        }
    }
}
