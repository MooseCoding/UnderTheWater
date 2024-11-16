package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class NewValues extends LinearOpMode {
    private DcMotorEx fL, fR, bL, bR;
    private DcMotorEx iM, oM1, oM2;

    @Override
    public void runOpMode() throws InterruptedException {
        fL = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
        bL = (DcMotorEx) hardwareMap.dcMotor.get("backLeft");
        fR = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
        bR = (DcMotorEx) hardwareMap.dcMotor.get("backRight");
        iM = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        oM1 = (DcMotorEx) hardwareMap.dcMotor.get("outtake1");
        oM2 = (DcMotorEx) hardwareMap.dcMotor.get("outtake2");

        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);
        iM.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            // Drivetrain
            double max;

            double axial = gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yaw = -gamepad1.right_stick_x;

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
            if(gamepad1.cross) {
                iM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (gamepad1.square) {
                oM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                oM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            telemetry.addData("om", oM1.getCurrentPosition());
            telemetry.addData("im", iM.getCurrentPosition());
            telemetry.update();
        }
    }
}
