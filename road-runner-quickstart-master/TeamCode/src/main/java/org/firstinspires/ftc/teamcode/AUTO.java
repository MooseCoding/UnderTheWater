package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class AUTO extends LinearOpMode {
    private DcMotorEx fL, fR, bL, bR;

    @Override
    public void runOpMode() throws InterruptedException {
        fL = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
        bL = (DcMotorEx) hardwareMap.dcMotor.get("backLeft");
        fR = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
        bR = (DcMotorEx) hardwareMap.dcMotor.get("backRight");
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive() && !isStopRequested())  {
            fR.setPower(0.2);
            fL.setPower(0.2);
            bL.setPower(0.2);
            bR.setPower(0.2);
        }
    }
}
