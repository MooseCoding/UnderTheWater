package org.firstinspires.ftc.teamcode.odometry;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;



@TeleOp
@Photon
public class OdometryTester extends LinearOpMode {
    double width = 18;
    double offset = -7;
    double iA = 0;
    double sX = 0;
    double sY = 0;

    private DcMotorEx BR, FR, BL, FL, PERP, LEFT;

    double[] currentPosition = new double[3];

    private OdometryController odometer;
    public void runOpMode() {
        BR = hardwareMap.get(DcMotorEx .class, "right");
        FR = hardwareMap.get(DcMotorEx.class, "fr");
        BL = hardwareMap.get(DcMotorEx.class, "bl");
        FL = hardwareMap.get(DcMotorEx.class, "fl");
        LEFT = hardwareMap.get(DcMotorEx.class, "left");
        PERP = hardwareMap.get(DcMotorEx.class, "perp");
        LEFT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PERP.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
        odometer = new OdometryController(width, offset, iA, sX, sY, LEFT, PERP, BR);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            currentPosition = odometer.update();
            telemetry.addData("right_odometer", BR.getCurrentPosition());
            telemetry.addData("center_odometer", PERP.getCurrentPosition());
            telemetry.addData("left_odometer", LEFT.getCurrentPosition());
            telemetry.addData("X_POS", currentPosition[0]);
            telemetry.addData("Y_POS", currentPosition[1]);
            telemetry.addData("ANGLE", currentPosition[2]);
            telemetry.update();
        }
    }
}
