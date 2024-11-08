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
public class Main extends LinearOpMode {
    private final Scalar lowerBlue = new Scalar(100, 150, 50);
    private final Scalar upperBlue = new Scalar(130,255,255);
    private final Scalar lowerRed1 = new Scalar(0,150,50);
    private final Scalar upperRed1 = new Scalar(10,255,255);
    private final Scalar lowerRed2 = new Scalar(170,150,50);
    private final Scalar upperRed2 = new Scalar(180,255,255);
    private final Scalar lowerYellow = new Scalar(20,150,50);
    private final Scalar upperYellow = new Scalar(30,255,255);
    private DcMotorEx fL, fR, bL, bR;
    private DcMotorEx liftMotor, armMotor;
    private Servo c1, c2, yawServo, pitchServo;

    private enum CLAW_STATE {
        STARTING,
        INTAKE,
        GRAB,
        OUTTAKE,
        HANG
    }

    private boolean claw_state_switched = false;

    private Gamepad g = new Gamepad();
    private CLAW_STATE current_claw_state;

    private double[] servos_zero = {0,0,0,0};

    private double[] intake_positions = {50,400,0,1,0,0}; // in order: arm motor -> lift motor -> pitch servo -> yaw servo -> c1 -> c2
    private double[] intake_power = {1,0.7};
    private double[] grab_positions = {-63,400,-0.6,1,-1,1};
    private double[] grab_power = {0.7,0.7};
    private double[] outtake_positions = {750,400,0.6,0.333,0,0};
    private double[] outtake_power = {0.7,0.7};

    private double[] hang_positions = {800, 400};
    private double[] hang_positions_final = {900, -12500};

    private OpenCvCamera camera;
    private int cID;
    
    private Sample currentTarget = new Sample();

    double t = -1;

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


                if(rotatedRect.size.height > 100 && rotatedRect.size.width > 100) {
                    for (int j = 0; j < 4; j++) {
                        Imgproc.line(img, rectPoints[j], rectPoints[(j + 1) % 4], new Scalar(0, 255, 0), 2);
                    }
                }

                if (samples.isEmpty()) {
                    samples.add(new Sample(rotatedRect, color));
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


    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStopRequested()) {
            fL = (DcMotorEx) hardwareMap.dcMotor.get("fl");
            bL = (DcMotorEx) hardwareMap.dcMotor.get("bl");
            fR = (DcMotorEx) hardwareMap.dcMotor.get("fr");
            bR = (DcMotorEx) hardwareMap.dcMotor.get("br");
            armMotor = (DcMotorEx) hardwareMap.dcMotor.get("armMotor");
            liftMotor = (DcMotorEx) hardwareMap.dcMotor.get("liftMotor");

            c1 = hardwareMap.servo.get("leftServo");
            c2 = hardwareMap.servo.get("rightServo");
            yawServo = hardwareMap.servo.get("yawServo");
            pitchServo = hardwareMap.servo.get("pitchServo");

            current_claw_state = CLAW_STATE.STARTING;

            bR.setDirection(DcMotorSimple.Direction.REVERSE);
            fR.setDirection(DcMotorSimple.Direction.REVERSE);
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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
                    findRectanglesByColor(frame, lowerYellow, upperYellow, Color.YELLOW);

                    return null;
                }

                @Override
                public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

                }
            });

            mVP = mVPB.build();

            waitForStart();

            while (opModeIsActive()) {
                double max;

                double axial = -gamepad1.left_stick_y;
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;

                double leftFrontPower = (axial + lateral) + yaw;
                double rightFrontPower = (axial - lateral) - yaw;
                double leftBackPower = (axial - lateral) + yaw;
                double rightBackPower = (axial + lateral) - yaw;


                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }

                fL.setPower(leftFrontPower);
                bL.setPower(leftBackPower);
                bR.setPower(rightBackPower);
                fR.setPower(rightFrontPower);

                // FSM
                switch(current_claw_state) {
                    case STARTING: {
                        armMotor.setTargetPosition((int)intake_positions[0]);
                        liftMotor.setTargetPosition((int)intake_positions[1]);
                        pitchServo.setPosition(intake_positions[2]);
                        yawServo.setPosition(intake_positions[3]);
                        c1.setPosition(intake_positions[4]);
                        c2.setPosition(intake_positions[5]);

                        armMotor.setPower(intake_power[0]);
                        liftMotor.setPower(intake_power[1]);

                        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        claw_state_switched = false;

                        current_claw_state = CLAW_STATE.INTAKE;
                        break;
                    }
                    case GRAB: {
                        if (gamepad1.right_trigger > 0) {
                            pitchServo.setPosition(grab_positions[2]);
                            yawServo.setPosition(grab_positions[3]);
                            c1.setPosition(grab_positions[4]);
                            c2.setPosition(grab_positions[5]);

                            if(t == -1) {
                            t = getRuntime(); }
                        }

                        if (t-getRuntime() > 1) {
                            current_claw_state = CLAW_STATE.OUTTAKE;
                        }

                        break;
                    }
                    case INTAKE: {
                        if(armMotor.getCurrentPosition() <= intake_positions[0] + 20 ) {
                            claw_state_switched = true;
                        }

                        if(claw_state_switched && gamepad1.a) {
                            claw_state_switched = false;
                            current_claw_state = CLAW_STATE.GRAB;
                        }

                        break;
                    }
                    case OUTTAKE: {
                        if(armMotor.getTargetPosition() == intake_positions[0] && t < 1) {
                            if (t == -1) {
                                t = getRuntime();
                            }

                            pitchServo.setPosition(outtake_positions[2]);
                            yawServo.setPosition(outtake_positions[3]);
                        }
                        if (t - getRuntime() > 1) {
                            t = -1;
                            armMotor.setTargetPosition((int)outtake_positions[0]);
                            liftMotor.setTargetPosition((int)outtake_positions[1]);

                            armMotor.setPower(outtake_power[0]);
                            liftMotor.setPower(outtake_power[1]);

                            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }

                        if(armMotor.getTargetPosition() == outtake_positions[0]) {
                            if(gamepad1.a) {
                                c1.setPosition(outtake_positions[4]);
                                c2.setPosition(outtake_positions[5]);
                                current_claw_state = CLAW_STATE.STARTING;
                            }
                        }

                        break;
                    }
                    case HANG: {

                        break;
                    }
                }

                if (gamepad1.dpad_up) {
                    current_claw_state = CLAW_STATE.HANG;
                }

                telemetry.addData("fLP",leftFrontPower);
                telemetry.addData("fRP", rightFrontPower);
                telemetry.addData("bLP", leftBackPower);
                telemetry.addData("bRP", rightBackPower);
                telemetry.addData("arm motor", armMotor.getCurrentPosition());
                telemetry.addData("arm motor target", armMotor.getTargetPosition());
                telemetry.addData("lift motor", liftMotor.getCurrentPosition());
                telemetry.addData("lift motor target", liftMotor.getTargetPosition());
                telemetry.addData("current state", current_claw_state);
                telemetry.update();
                g.copy(gamepad1);
            }
        }
    }
}
