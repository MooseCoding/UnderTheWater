package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Config
public class DriveMotorFinder extends OpMode {
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
        if(gamepad1.cross){
            bL.setPower(0.3);
        }
        else {
            bL.setPower(0);
        }

         if(gamepad1.triangle) {
            fL.setPower(0.3);
        }
         else {
             fL.setPower(0);
         }


         if(gamepad1.circle) {
            bR.setPower(0.3);
        }
         else {
             bR.setPower(0);
         }

         if(gamepad1.square) {
             fR.setPower(0.3);
         }
         else {
             fR.setPower(0);
         }
    }
}
