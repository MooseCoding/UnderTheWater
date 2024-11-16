package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Lift {
    private DcMotorEx  oM1, oM2;
    private Servo outtakeClaw1, outtakeClaw2, outtakePitch;


    public Lift(HardwareMap hardwareMap) {
        oM1 = (DcMotorEx) hardwareMap.dcMotor.get("outtake1");
        oM2 = (DcMotorEx) hardwareMap.dcMotor.get("outtake2");
        outtakeClaw1 = hardwareMap.servo.get("gS1");
        outtakeClaw2 = hardwareMap.servo.get("gS2");
        outtakePitch = hardwareMap.servo.get("gPS");
        oM2.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public boolean good(int pos) {
        return oM1.getCurrentPosition() > pos - 50;
    }

    public boolean good2(int pos) {
        return oM1.getCurrentPosition() < pos - 50;
    }

    public void DriveLift(double power, int position) {
        oM1.setTargetPosition(position);
        oM2.setTargetPosition(position);

        oM1.setPower(power);
        oM2.setPower(power);

        oM1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        oM2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void holdSample() {
        outtakePitch.setPosition(0.5);
        outtakeClaw1.setPosition(-0.3);
        outtakeClaw2.setPosition(0.45);
    }

    public void dropSample() {
        outtakePitch.setPosition(0.5);
        outtakeClaw1.setPosition(0.45);
        outtakeClaw2.setPosition(-1);
    }
    public void returnClaw() {
        outtakeClaw1.setPosition(0.45);
        outtakeClaw2.setPosition(-1);
        outtakePitch.setPosition(0.0);
    }


}


