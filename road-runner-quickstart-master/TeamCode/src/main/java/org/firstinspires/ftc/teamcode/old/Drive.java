package org.firstinspires.ftc.teamcode.old;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Config
@Disabled
public class Drive extends OpMode {
    private DcMotorEx fL, bR, bL, fR;
    public static double multi = 1;

    @Override
    public void init() {
        fL = hardwareMap.get(DcMotorEx.class, "frontLeft");
        bR = hardwareMap.get(DcMotorEx.class, "backRight");
        bL = hardwareMap.get(DcMotorEx.class, "backLeft");
        fR = hardwareMap.get(DcMotorEx.class, "frontRight");

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
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

        fL.setPower(leftFrontPower*multi);
        bL.setPower(leftBackPower*multi);
        bR.setPower(rightBackPower*multi);
        fR.setPower(rightFrontPower*multi);
    }
}
