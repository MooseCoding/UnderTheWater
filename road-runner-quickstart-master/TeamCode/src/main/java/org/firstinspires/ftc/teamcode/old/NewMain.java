package org.firstinspires.ftc.teamcode.old;

import android.graphics.Canvas;
import android.util.Size;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.robot.vision.Color;
import org.firstinspires.ftc.teamcode.robot.vision.Sample;
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


@Photon
@TeleOp
@Disabled
public class NewMain extends LinearOpMode {
    private final Scalar lowerBlue = new Scalar(100, 150, 50);
    private final Scalar upperBlue = new Scalar(130,255,255);
    private final Scalar lowerRed1 = new Scalar(0,150,50);
    private final Scalar upperRed1 = new Scalar(10,255,255);
    private final Scalar lowerRed2 = new Scalar(170,150,50);
    private final Scalar upperRed2 = new Scalar(180,255,255);
    private final Scalar lowerYellow = new Scalar(20,150,50);
    private final Scalar upperYellow = new Scalar(30,255,255);

    private DcMotorEx fL, fR, bL, bR;
    private DcMotorEx iM, oM1, oM2;
    private Servo c1, c2, yawServo, pitchServo;
    private Servo gC1, gC2, gPS; 

    private enum CLAW_STATE {
        STARTING,
        INTAKE,
        GRAB,
        OUTTAKE,
        HANG
    }

    private boolean claw_state_switched = false;

    private double t = -1; 

    private Gamepad g = new Gamepad();
    private CLAW_STATE current_claw_state;

    private double[] homes = {0,0,  0,0,0}; //intake, lift, gps, gc1, gc2, 
    private double[] homes_power = {0.7,1};
    private double[] intake_positions = {1500,0.6,0.55,  0,1}; // in order: intake -> pitch servo -> yaw servo -> c1 -> c2
    private double[] intake_power = {0.8}; 

    private double[] grab_positions = {0,0.55,1,-1}; //  pitch -> yaw -> c1 -> c2

    private double[] trans_positions = {0,  1,-1,  0 ,0.55,0,-1  , 0}; // gPS -> gC1 -> gC2 -> pitch -> yaw -> c1 -> c2 -> lift

    private double[] outtake_positions = {3500,0.6428  ,0,0}; // 3850,  lift -> pitch (grab) -> c1 (g) -> c2 (g)
    private double[] outtake_power = {1};
 
    private OpenCvCamera camera;
    private int cID;

    public static ArrayList<Sample> samples = new ArrayList<>();

    public void DriveLift(int position, double power) {
        oM1.setTargetPosition(position);
        oM2.setTargetPosition(position);

        oM1.setPower(power);
        oM2.setPower(power);

        oM1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        oM2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void ExtendIntake(int position, double power) {
        iM.setTargetPosition(position);
        iM.setPower(power);
        iM.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
    }

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

    private enum INSTATE_GRAB {
        WAIT, 
        GCS,
        CS,
        GCP,
    }

    private INSTATE_GRAB ig = INSTATE_GRAB.WAIT; 

    private enum INSTATE_OUTTAKE {
        WAIT,
        LIFT, 
        TURN,
        DROP,
    }

    private INSTATE_OUTTAKE io = INSTATE_OUTTAKE.WAIT; 

    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStopRequested()) {
            fL = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
            bL = (DcMotorEx) hardwareMap.dcMotor.get("backLeft");
            fR = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
            bR = (DcMotorEx) hardwareMap.dcMotor.get("backRight");
            iM = (DcMotorEx) hardwareMap.dcMotor.get("intake");
            oM1 = (DcMotorEx) hardwareMap.dcMotor.get("outtake1");
            oM2 = (DcMotorEx) hardwareMap.dcMotor.get("outtake2");

            
            c1 = hardwareMap.servo.get("leftServo");
            c2 = hardwareMap.servo.get("rightServo");
            yawServo = hardwareMap.servo.get("yawServo");
            pitchServo = hardwareMap.servo.get("pitchServo");

            gC1 = hardwareMap.servo.get("gS1");
            gC2 = hardwareMap.servo.get("gS2");
            gPS = hardwareMap.servo.get("gPS"); 
            
            current_claw_state = CLAW_STATE.STARTING;

            fR.setDirection(DcMotorSimple.Direction.REVERSE);
            bL.setDirection(DcMotorSimple.Direction.REVERSE);
            bR.setDirection(DcMotorSimple.Direction.REVERSE);
            iM.setDirection(DcMotorSimple.Direction.REVERSE);
            oM2.setDirection(DcMotorSimple.Direction.REVERSE);
            
            iM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            oM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            oM2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            oM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            oM2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Init the vision portal for the camera
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
                // Drivetrain
                double max;

                double axial = gamepad1.left_stick_y;
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

                // FSM for the claw
                switch(current_claw_state) {
                    case STARTING: {
                        if(gamepad1.right_bumper) {
                            ExtendIntake((int) intake_positions[0], intake_power[0]);
                            
                            pitchServo.setPosition(intake_positions[1]);
                            yawServo.setPosition(intake_positions[2]);
                            c1.setPosition(intake_positions[3]);
                            c2.setPosition(intake_positions[4]);

                            current_claw_state = CLAW_STATE.INTAKE; 
                        }
                        break;
                    }
                    case INTAKE: {
                            if (gamepad1.right_trigger > 0 && t == -1) {
                                c1.setPosition(grab_positions[2]);
                                c2.setPosition(grab_positions[3]);

                                t = getRuntime(); 
                            }
                            else if(gamepad1.left_trigger > 0) {
                                current_claw_state = CLAW_STATE.STARTING;
                            }

                            if (t != -1 && getRuntime() - t > 1) {
                                pitchServo.setPosition(grab_positions[0]);
                                t = -1; 

                                DriveLift((int) trans_positions[7], homes_power[1]);
                                ExtendIntake((int) homes[0], homes_power[0]);
                                current_claw_state = CLAW_STATE.GRAB; 
                            }
                        break;
                    }
                    case GRAB: {
                        switch(ig) {
                            case WAIT: {
                                if (iM.getCurrentPosition() <= homes[0] + 20) {
                                    DriveLift((int) homes[1], homes_power[1]);
                                    ig = INSTATE_GRAB.GCS; 
                                }
                                break; 
                            }
                            case GCS: {
                                if(t == -1) {
                                    gC1.setPosition(trans_positions[1]);
                                    gC2.setPosition(trans_positions[2]);

                                    t = getRuntime(); 
                                }

                                if (t != -1 && getRuntime() - t > 0.2 ) {
                                    t = -1; 
                                    ig = INSTATE_GRAB.CS; 
                                }

                                break;
                            }
                            case CS: {
                                if (t == -1) {
                                    c1.setPosition(trans_positions[5]);
                                    c2.setPosition(trans_positions[6]);

                                    t = getRuntime(); 
                                }

                                if (t != -1 && getRuntime() - t > 0.2) {
                                    t = -1;
                                    ig = INSTATE_GRAB.GCP; 
                                }
                                break;
                            }
                            case GCP: {
                                if (t == -1) {
                                    gPS.setPosition(trans_positions[0]); 

                                    t = getRuntime(); 
                                }
                                if (t != -1 && getRuntime() - t > 0.2) {
                                    t = -1;
                                    current_claw_state = CLAW_STATE.OUTTAKE; 
                                    ig = INSTATE_GRAB.WAIT; 
                                }
                                break;
                            } 
                        }

                        break;
                    }
                    case OUTTAKE: {
                        switch (io) {
                            case WAIT: {
                                if(gamepad1.cross) {
                                    io = INSTATE_OUTTAKE.LIFT;
                                }
                                break; 
                            } 
                            case LIFT: {
                                DriveLift((int) outtake_positions[0], outtake_power[0]);

                                if(oM1.getCurrentPosition() >= outtake_positions[0] - 30) {
                                    io = INSTATE_OUTTAKE.TURN;
                                }
                                break;
                            }
                            case TURN: {
                                if (t == -1) {
                                    gPS.setPosition(outtake_positions[1]); 

                                    t = getRuntime(); 
                                }
                                if (t != -1 && getRuntime() - t > 0.2) {
                                    t = -1;
                                    io = INSTATE_OUTTAKE.DROP; 
                                }
                                break;
                            }
                            case DROP: {
                                if (t == -1) {
                                    if(gamepad1.cross) {
                                        gC1.setPosition(outtake_positions[2]);
                                        gC2.setPosition(outtake_positions[3]);

                                        t = getRuntime();
                                    }
                                }
                                if (t != -1 && getRuntime() - t > 0.2) {
                                    t = -1;

                                    DriveLift((int) homes[1], homes_power[1]);
                                    gPS.setPosition(homes[2]);
                                    gC1.setPosition(homes[3]);
                                    gC2.setPosition(homes[4]); 

                                    io = INSTATE_OUTTAKE.WAIT; 

                                    current_claw_state = CLAW_STATE.STARTING; 
                                }
                                break;
                            }
                        }
                        break;
                    }
                }

                if (gamepad1.dpad_up){
                    current_claw_state = CLAW_STATE.HANG;
                }

                telemetry.addData("iM", iM.getCurrentPosition());
                telemetry.addData("oM1", oM1.getCurrentPosition());
                telemetry.addData("oM2", oM2.getCurrentPosition());
                telemetry.update();
                g.copy(gamepad1);
            }
        }
    }
}
