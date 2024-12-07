package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hang {
    public static DcMotorEx hM;
    private PIDFController controller;
    public static double f = 0.0, d = 0.0, i = 0.0, p = 0.0;

    public static double target = 0.0;
    public Hang(HardwareMap hardwareMap) {
        controller = new PIDFController(p,i,d,f);
        hM = hardwareMap.get(DcMotorEx.class, "hangMotor");
    }

    public void update() {
        hM.setPower(controller.calculate(hM.getCurrentPosition(), target));
    }

    public void hangUp() {
        target = 1000;
    }

    public void hangDown() {
        target = 100;
    }
}
