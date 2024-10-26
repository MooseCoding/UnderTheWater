package org.firstinspires.ftc.teamcode;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@Photon
@TeleOp
public class Leah extends LinearOpMode {
    private DcMotorEx fL, fR, bL, bR;
    private DcMotorEx liftMotor, armMotor;
    private CRServo c1, c2, yawServo, pitchServo;
    private Gamepad g;

    public void runOpMode() throws InterruptedException {
        while (!isStopRequested()) {
            fL = (DcMotorEx) hardwareMap.dcMotor.get("frontLeftMotor");
            bL = (DcMotorEx) hardwareMap.dcMotor.get("backLeftMotor");
            fR = (DcMotorEx) hardwareMap.dcMotor.get("frontRightMotor");
            bR = (DcMotorEx) hardwareMap.dcMotor.get("backRightMotor");
            armMotor = (DcMotorEx) hardwareMap.dcMotor.get("armMotor");
            liftMotor = (DcMotorEx) hardwareMap.dcMotor.get("liftMotor");

            c1 = hardwareMap.crservo.get("leftServo");
            c2 = hardwareMap.crservo.get("rightServo");
            yawServo = hardwareMap.crservo.get("yawServo");
            pitchServo = hardwareMap.crservo.get("pitchServo");

            bR.setDirection(DcMotorSimple.Direction.REVERSE);
            fR.setDirection(DcMotorSimple.Direction.REVERSE);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        waitForStart();

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
        }
        g.copy(gamepad1);
    }
}
