package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class robotPipeline extends OpenCvPipeline {
    public static ArrayList<Sample> samples = new ArrayList<>();
    private final Scalar lowerBlue = new Scalar(100, 150, 50);
    private final Scalar upperBlue = new Scalar(130,255,255);
    private final Scalar lowerRed1 = new Scalar(0,150,50);
    private final Scalar upperRed1 = new Scalar(10,255,255);
    private final Scalar lowerRed2 = new Scalar(170,150,50);
    private final Scalar upperRed2 = new Scalar(180,255,255);
    private final Scalar lowerYellow = new Scalar(20,150,50);
    private final Scalar upperYellow = new Scalar(30,255,255);

    @Override
    public Mat processFrame(Mat input) {
        findRectanglesByColor(input, lowerBlue, upperBlue, Color.BLUE);
        findRectanglesByColor(input, lowerRed1, upperRed1, Color.RED);
        findRectanglesByColor(input, lowerRed2, upperRed2, Color.RED);
        findRectanglesByColor(input, lowerYellow, upperYellow, Color.YELLOW);

        return input;
    }

    public static void findRectanglesByColor(Mat img, Scalar lowerBound, Scalar upperBound, Color color) {
        // Reuse Mats instead of creating new ones
        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2HSV); // Convert directly within the original Mat

        // Apply threshold within the existing img Mat
        Core.inRange(img, lowerBound, upperBound, img); // Now `img` serves as a mask

        // Find contours directly on the modified `img` (now the mask)
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(img, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Iterate through contours and filter for rectangles
        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);

            if (isRectangle(rotatedRect)) {
                // Draw rectangle directly on the original image
                Point[] rectPoints = new Point[4];
                rotatedRect.points(rectPoints);

                for (int j = 0; j < 4; j++) {
                    Imgproc.line(img, rectPoints[j], rectPoints[(j + 1) % 4], new Scalar(0, 255, 0), 2);
                }

                samples.add(new Sample(rotatedRect, color));
            }
        }
    }

    // Helper method to check if a contour is roughly a rectangle
    private static boolean isRectangle(RotatedRect rotatedRect) {
        double aspectRatio = Math.min(rotatedRect.size.width, rotatedRect.size.height) /
                             Math.max(rotatedRect.size.width, rotatedRect.size.height);
        return aspectRatio > 0.8 && aspectRatio < 1.2;  // roughly square
    }
}
