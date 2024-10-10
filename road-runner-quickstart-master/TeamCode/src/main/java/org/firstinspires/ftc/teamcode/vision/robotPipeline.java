package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

class robotPipeline extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {
        return input;
    }

    private void draw_rectangles(Mat input, Mat[] mats, int[][] cords) {
        for (Mat m : mats) {
            Imgproc.rectangle(input, );
        }
    }
}