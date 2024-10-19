package org.firstinspires.ftc.teamcode;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo; 
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
public class Main extends LinearOpMode {
    private DcMotorEx fL, fR, bL, bR;
    private DcMotorEx liftMotor, armMotor;
    private CRServo c1, c2, yawServo, pitchServo;

    private enum CLAW_STATE {
        STARTING,
        INTAKE,
        GRAB,
        OUTTAKE,
    }

    private boolean claw_state_switched = false;

    private Gamepad g = new Gamepad();
    private CLAW_STATE current_claw_state;

    private double[] servos_zero = {0,0,0,0};

    private double[] intake_positions = {0,0,0,0,0,0}; // in order: arm motor -> lift motor -> pitch servo -> yaw servo -> c1 -> c2
    private double[] intake_power = {0,0,0,0,0,0};
    private double[] grab_positions = {0,0,0,0,0,0};
    private double[] grab_power = {0,0,0,0,0,0};
    private double[] outtake_positions = {0,0,0,0,0,0};
    private double[] outtake_power = {0,0,0,0,0,0};
    private int[] servo_positions = {0,0,0,0}; // pitch servo  -> yaw servo -> c1 -> c2

    private OpenCvCamera camera;
    private int cID;
    
    private Sample currentTarget = new Sample();

    private void clawToAngle(double angle) {
//        yawServo.getController().setServoPosition(servo_positions[1], servos_zero[1]);
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

            c1 = hardwareMap.crservo.get("leftServo");
            c2 = hardwareMap.crservo.get("rightServo");
            yawServo = hardwareMap.crservo.get("yawServo");
            pitchServo = hardwareMap.crservo.get("pitchServo");

            current_claw_state = CLAW_STATE.STARTING;

            bR.setDirection(DcMotorSimple.Direction.REVERSE);
            fR.setDirection(DcMotorSimple.Direction.REVERSE);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//            servo_positions[0] = pitchServo.getPortNumber();
//            servo_positions[1] = yawServo.getPortNumber();
//            servo_positions[2] = c1.getPortNumber();
//            servo_positions[3] = c2.getPortNumber();

            cID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()); // camera id
            camera = OpenCvCameraFactory.getInstance().createWebcam( hardwareMap.get(WebcamName.class, "cam"), cID); // camera object
            camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
            camera.setPipeline(new robotPipeline());

            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT); // CHANGE DEPENDING ON WHAT IT LOOKS LIKE
                }
                @Override
                public void onError(int errorCode)
                {
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });

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
//                        pitchServo.getController().setServoPosition(servo_positions[0], intake_positions[2]);
//                        yawServo.getController().setServoPosition(servo_positions[1], intake_positions[3]);
//                        c1.getController().setServoPosition(servo_positions[2], intake_positions[4]);
//                        c2.getController().setServoPosition(servo_positions[3], intake_positions[5]);

                        armMotor.setPower(intake_power[0]);
                        liftMotor.setPower(intake_power[1]);
                        pitchServo.setPower(intake_power[2]);
                        yawServo.setPower(intake_power[3]);
                        c1.setPower(intake_power[4]);
                        c2.setPower(intake_power[5]);

                        current_claw_state = CLAW_STATE.INTAKE;
                        break;
                    }
                    case INTAKE: {
                        if(armMotor.getCurrentPosition() >= intake_positions[0]) {
                            claw_state_switched = true;
                        }

                        if(claw_state_switched && !g.a && gamepad1.a) {
                            claw_state_switched = false;
                        }

                        break;
                    }
                    case GRAB: {
                        for (Sample s : robotPipeline.samples) {
                            if (s.color == Color.YELLOW) {
                                currentTarget = s;
                            }
                        }



                        break;
                    }
                    case OUTTAKE: {

                    }
                }

                telemetry.addData("fLP",leftFrontPower);
                telemetry.addData("fRP", rightFrontPower);
                telemetry.addData("bLP", leftBackPower);
                telemetry.addData("bRP", rightBackPower);
                telemetry.update();
                g.copy(gamepad1);
            }
        }
    }
}
