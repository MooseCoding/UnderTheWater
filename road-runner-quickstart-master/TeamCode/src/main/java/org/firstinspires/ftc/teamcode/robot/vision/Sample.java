package org.firstinspires.ftc.teamcode.robot.vision;

import org.opencv.core.RotatedRect;

public class Sample {
    public RotatedRect rect;
    public Color color;

    public Sample(RotatedRect rect, Color color) {
        this.rect = rect;
        this.color = color;
    }

    public Sample() {

    }
}