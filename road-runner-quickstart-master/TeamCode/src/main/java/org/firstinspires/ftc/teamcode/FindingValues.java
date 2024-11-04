package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.util.Size;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.vision.Color;
import org.firstinspires.ftc.teamcode.vision.Sample;
import org.firstinspires.ftc.teamcode.vision.robotPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;




import java.util.ArrayList;

@Photon
@TeleOp
public class FindingValues extends LinearOpMode {
    private final Scalar lowerBlue = new Scalar(100, 150, 50);
    private final Scalar upperBlue = new Scalar(130,255,255);
    private final Scalar lowerRed1 = new Scalar(0,150,50);
    private final Scalar upperRed1 = new Scalar(10,255,255);
    private final Scalar lowerRed2 = new Scalar(170,150,50);
    private final Scalar upperRed2 = new Scalar(180,255,255);
    private final Scalar lowerYellow = new Scalar(20,150,50);
    private final Scalar upperYellow = new Scalar(30,255,255);
    private Gamepad g = new Gamepad();
    private DcMotor liftMotor, armMotor;
    private Servo c1, c2, yawServo, pitchServo;
    private OpenCvCamera camera;
    private int cID;
    public static ArrayList<Sample> samples = new ArrayList<>();

    public static void findRectanglesByColor(Mat img, Scalar lowerBound, Scalar upperBound, Color color) {
        // Convert the image to HSV
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(img, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Threshold the HSV image to get only specific colors
        Mat mask = new Mat();
        Core.inRange(hsvImage, lowerBound, upperBound, mask);

        // Find contours
        ArrayList<MatOfPoint> contours = new ArrayList<>();
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

    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStopRequested()) {
            //armMotor =hardwareMap.dcMotor.get("armMotor");
            //liftMotor =hardwareMap.dcMotor.get("liftMotor");

            //c1 = hardwareMap.servo.get("leftServo");
            //c2 = hardwareMap.servo.get("rightServo");
            //yawServo = hardwareMap.servo.get("yawServo");
            //pitchServo = hardwareMap.servo.get("pitchServo");

            //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            // Initalize the Camera, and Camera ID
            /*
            cID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()); // camera id
            camera = OpenCvCameraFactory.getInstance().createWebcam( hardwareMap.get(WebcamName.class, "cam"), cID); // camera object

            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT); // CHANGE DEPENDING ON WHAT IT LOOKS LIKE
                    camera.setPipeline(new robotPipeline());
                }
                @Override
                public void onError(int errorCode)
                {

                }
            });*/

            VisionPortal mVP ;
            VisionPortal.Builder mVPB = new VisionPortal.Builder();
            mVPB.setCamera(hardwareMap.get(WebcamName.class, "cam"))
                            .setCameraResolution(new Size(640, 480))
                                    .enableLiveView(true)
                                            .setAutoStopLiveView(true)
                                                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG);


            mVPB.addProcessors(new VisionProcessor() {
                @Override
                public void init(int width, int height, CameraCalibration calibration) {

                }

                @Override
                public Object processFrame(Mat frame, long captureTimeNanos) {
                    findRectanglesByColor(frame, lowerBlue, upperBlue, Color.BLUE);
                    findRectanglesByColor(frame, lowerRed1, upperRed1, Color.RED);
                    findRectanglesByColor(frame, lowerRed2, upperRed2, Color.RED);
                    findRectanglesByColor(frame, lowerYellow, upperYellow, Color.YELLOW);

                    return null;
                }

                @Override
                public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

                }
            });

            mVP = mVPB.build();

            waitForStart();
            while(opModeIsActive()) {
                if (gamepad1.a && !g.a) {
                    c1.setPosition(0);
                    c2.setPosition(1);
                    yawServo.setPosition(0.9);
                    pitchServo.setPosition(0.2);
                }
                if (gamepad1.b && !g.b) {
                    c1.setPosition(0.3);
                    c2.setPosition(0.7);
                    yawServo.setPosition(0.5);
                    pitchServo.setPosition(-0.2);
                }



                /*
                    if (gamepad1.x && !g.x) {
                        yawServo.setPosition(0);
                    }
                    if (gamepad1.y && !g.y) {
                        pitchServo.setPosition(0);
                    }
                 */

                /*if(gamepad1.right_trigger > 0) {
                    armMotor.setTargetPosition(-600);
                    liftMotor.setTargetPosition(6000);
                    armMotor.setPower(0.5);
                    liftMotor.setPower(1);
                    //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                else if (gamepad1.left_trigger > 0) {
                    liftMotor.setPower(-1);
                }
                else {
                     liftMotor.setPower(0);
                }

                if (gamepad1.dpad_up) {
                    liftMotor.setTargetPosition(6000);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    liftMotor.setPower(0.5);
                }

                if (g.a) {
                    armMotor.setTargetPosition(-1300);

                }*/

                telemetry.addData("aM", armMotor.getCurrentPosition());
                telemetry.addData("aMTa", armMotor.getTargetPosition());
                telemetry.update();

                g.copy(gamepad1);
            }
        }
    }
}
