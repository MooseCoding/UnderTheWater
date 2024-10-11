package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.RotatedRect;

public class Sample {
    public RotatedRect rect;
    public Color color;

    Sample(RotatedRect rect, Color color) {
        this.rect = rect;
        this.color = color;
    }

    public Sample() {

    }
}