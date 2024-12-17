package org.firstinspires.ftc.teamcode.old;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Photon
@Autonomous
@Disabled
public class Auto extends OpMode {
    private DcMotorEx fL, fR, bL, bR;

    @Override
    public void init() {
        fL = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
        bL = (DcMotorEx) hardwareMap.dcMotor.get("backLeft");
        fR = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
        bR = (DcMotorEx) hardwareMap.dcMotor.get("backRight");

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        fL.setPower(0.4);
        fR.setPower(0.4);
        bL.setPower(0.4);
        bR.setPower(0.4);
    }
}
