package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@Disabled
public class Hang {
    public static DcMotorEx hM;
    private PIDFController controller;
    public static double f = 0.0, d = 0.0, i = 0.0, p = 0.0;

    public static double target = 0.0;

    public static boolean pidfused = true;
    public Hang(HardwareMap hardwareMap) {
        controller = new PIDFController(p,i,d,f);
        hM = hardwareMap.get(DcMotorEx.class, "hang");
    }

    public void update() {
        if(pidfused) {
            hM.setTargetPosition((int) target);
            hM.setPower(0.3);
            hM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void hangUp() {
        target = -1500;
    }

    public void hangDown() {
        target = 500;
    }
}
