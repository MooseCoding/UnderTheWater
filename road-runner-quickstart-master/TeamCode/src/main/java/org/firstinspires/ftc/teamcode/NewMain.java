package org.firstinspires.ftc.teamcode;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.Color;
import org.firstinspires.ftc.teamcode.vision.Sample;
import org.firstinspires.ftc.teamcode.vision.robotPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Photon
@TeleOp
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

    private double[] intake_positions = {0, 0,0,0,0,0,0}; // in order: iM -> oM1 -> oM2 -> pitch servo -> yaw servo -> c1 -> c2
    private double[] intake_power = {0,0,0}; 
    private double[] grab_positions = {0,0,0,0,0,0,0};
    private double[] grab_power = {0,0,0}; 
    private double[] outtake_positions = {0,0,0,0,0,0,0};
    private double[] outtake_power = {0,0,0};
 
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
            
            current_claw_state = CLAW_STATE.STARTING;

            bR.setDirection(DcMotorSimple.Direction.REVERSE);
            fR.setDirection(DcMotorSimple.Direction.REVERSE);
            iM.setDirection(DcMotorSimple.Direction.REVERSE);
            oM2.setDirection(DcMotorSimple.Direction.REVERSE);
            
            iM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            oM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            oM2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

                // FSM for the claw
                switch(current_claw_state) {
                    case STARTING: {
                        iM.setTargetPosition(intake_positions[0]);
                        oM1.setTargetPosition(intake_positions[1]);
                        oM2.setTargetPosition(intake_positions[2]);

                        iM.setPower(intake_power[0]);
                        oM1.setPower(intake_power[1]);
                        oM2.setPower(intake_power[2]);

                        iM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        oM1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        oM2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        pitchServo.setPosition(intake_positions[3]);
                        yawServo.setPosition(intake_positions[4]);
                        c1.setPosition(intake_positions[5]);
                        c2.setPosition(intake_positions[6]); 

                        break;
                    }
                    case INTAKE: {
                        iM.setTargetPosition(grab_positions[0]);
                        oM1.setTargetPosition(grab_positions[1]);
                        oM2.setTargetPosition(grab_positions[2]);

                        iM.setPower(grab_power[0]);
                        oM1.setPower(grab_power[1]);
                        oM2.setPower(grab_power[2]);

                        iM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        oM1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        oM2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        pitchServo.setPosition(grab_positions[3]);
                        yawServo.setPosition(grab_positions[4]);
                        c1.setPosition(grab_positions[5]);
                        c2.setPosition(grab_positions[6]); 

                        break;
                    }
                    case GRAB: {
                        iM.setTargetPosition(outtake_positions[0]);
                        oM1.setTargetPosition(outtake_positions[1]);
                        oM2.setTargetPosition(outtake_positions[2]);

                        iM.setPower(outtake_power[0]);
                        oM1.setPower(outtake_power[1]);
                        oM2.setPower(outtake_power[2]);

                        iM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        oM1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        oM2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        pitchServo.setPosition(outtake_positions[3]);
                        yawServo.setPosition(outtake_positions[4]);
                        c1.setPosition(outtake_positions[5]);
                        c2.setPosition(outtake_positions[6]); 

                        break;
                    }
                    case OUTTAKE: {
                        iM.setTargetPosition(intake_positions[0]);
                        oM1.setTargetPosition(intake_positions[1]);
                        oM2.setTargetPosition(intake_positions[2]);

                        iM.setPower(intake_power[0]);
                        oM1.setPower(intake_power[1]);
                        oM2.setPower(intake_power[2]);

                        iM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        oM1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        oM2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        pitchServo.setPosition(intake_positions[3]);
                        yawServo.setPosition(intake_positions[4]);
                        c1.setPosition(intake_positions[5]);
                        c2.setPosition(intake_positions[6]); 

                        break;
                    }
                }

                if (gamepad1.dpad_up){
                    current_claw_state = CLAW_STATE.HANG;
                }

                /*
                if(gamepad1.right_bumper) {
                    iM.setTargetPosition(1577);
                    iM.setPower(1);
                    iM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                else if (gamepad1.left_bumper){
                    iM.setTargetPosition(10);
                    iM.setPower(1);
                    iM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                if(gamepad1.triangle) {
                    oM2.setTargetPosition(4200);
                    oM2.setPower(1);
                    oM2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (gamepad1.cross) {
                    oM1.setTargetPosition(4200);
                    oM1.setPower(1);
                    oM1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if(gamepad1.square) {
                    oM2.setTargetPosition(0);
                    oM2.setPower(1);
                    oM2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (gamepad1.circle) {
                    oM1.setTargetPosition(0);
                    oM1.setPower(1);
                    oM1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }*/ 

                telemetry.addData("iM", iM.getCurrentPosition());
                telemetry.addData("oM1", oM1.getCurrentPosition());
                telemetry.addData("oM2", oM2.getCurrentPosition());
                telemetry.update();
                g.copy(gamepad1);
            }
        }
    }
}
