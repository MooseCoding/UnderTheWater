package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

private class Sample {
	private Point[] cords;
	private double theta;
	private int color; 

	Sample()
}

class robotPipeline extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {
        return input;
    }

    public static Sample[] findRectanglesByColor(Mat img, Scalar lowerBound, Scalar upperBound, String colorName) {
        // Convert the image to HSV
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(img, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Threshold the HSV image to get only specific colors
        Mat mask = new Mat();
        Core.inRange(hsvImage, lowerBound, upperBound, mask);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Iterate through contours and filter for rectangles
        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);

            // Get the dimensions of the rectangle
            if (isRectangle(rotatedRect)) {
                // Draw the rectangle
                Point[] rectPoints = new Point[4];
                rotatedRect.points(rectPoints);
                for (int j = 0; j < 4; j++) {
                    Imgproc.line(img, rectPoints[j], rectPoints[(j + 1) % 4], new Scalar(0, 255, 0), 2);
                }
            }
        }
    }

    // Helper method to check if a contour is roughly a rectangle
    private static boolean isRectangle(RotatedRect rotatedRect) {
        double aspectRatio = Math.min(rotatedRect.size.width, rotatedRect.size.height) /
                             Math.max(rotatedRect.size.width, rotatedRect.size.height);
        return aspectRatio > 0.8 && aspectRatio < 1.2;  // roughly square
    }


    private void draw_rectangles(Mat input, Mat[] mats, int[][] cords) {
        for (Mat m : mats) {
            Imgproc.rectangle(input, );
        }
    }
}
