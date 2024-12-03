package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.vision.Sample;

import java.util.ArrayList;

@TeleOp
public class NewTeleOP extends OpMode {
    private final Robot robot = new Robot();
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    private ArrayList<Sample> samples = new ArrayList<>();

    @Override
    public void loop() {
        robot.loop_func(gamepad1, samples, getRuntime());
    }
}
