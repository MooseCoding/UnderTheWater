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
public class Main extends LinearOpMode {
    private DcMotorEx fL, fR, bL, bR;
    private DcMotorEx liftMotor, armMotor;
    private Servo c1, c2, yawServo, pitchServo;

    private enum CLAW_STATE {
        STARTING,
        INTAKE,
        GRAB,
        OUTTAKE,
        HANG,
        HANG_FINAL,
    }

    private boolean claw_state_switched = false;

    private Gamepad g = new Gamepad();
    private CLAW_STATE current_claw_state;

    private double[] servos_zero = {0,0,0,0};

    private double[] intake_positions = {0,-6000,0.2,0.9,0,1}; // in order: arm motor -> lift motor -> pitch servo -> yaw servo -> c1 -> c2
    private double[] intake_power = {0,0};
    private double[] grab_positions = {0,0,0,0,0,0};
    private double[] grab_power = {0,0};
    private double[] outtake_positions = {-944,0,-0.2,0.5,0.3,0.7};
    private double[] outtake_power = {0,0};

    private double[] hang_positions = {-945, 0};

    private double[] hang_final_positions = {-600, -12000}; // arm motor -> lift motor
    private OpenCvCamera camera;
    private int cID;

    private double[] times = {0.6,0.6}; //intake time, outtake time
    private double time = 0.0;

    private Sample currentTarget = new Sample();

    /*
    private void clawToAngle(double angle) {
        yawServo.getController().setServoPosition(servo_positions[1], servos_zero[1]);
    }*/

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
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            /*
            cID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()); // camera id
            camera = OpenCvCameraFactory.getInstance().createWebcam( hardwareMap.get(WebcamName.class, "cam"), cID); // camera object
            camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT); // CHANGE DEPENDING ON WHAT IT LOOKS LIKE
                    camera.setPipeline(new robotPipeline());
                }
                @Override
                public void onError(int errorCode)
                {

                }
            });

            */

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


                        current_claw_state = CLAW_STATE.INTAKE;
                        break;
                    }
                    case INTAKE: {
                        if(!g.a && gamepad1.a) {
                            c1.setPosition(outtake_positions[4]);
                            c2.setPosition(outtake_positions[5]);

                            if (time == 0) {
                                time = getRuntime();
                            }
                        }

                        if (getRuntime() -time >= times[0] && time!=0) {
                            armMotor.setTargetPosition((int)outtake_positions[0]);
                            liftMotor.setTargetPosition((int)outtake_positions[1]);
                            pitchServo.setPosition(outtake_positions[2]);
                            yawServo.setPosition(outtake_positions[3]);

                            armMotor.setPower(outtake_power[0]);
                            liftMotor.setPower(outtake_power[1]);

                            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            current_claw_state = CLAW_STATE.OUTTAKE;
                        }

                        break;
                    }
                    /*
                    case GRAB: {
                        for (Sample s : robotPipeline.samples) {
                            if (s.color == Color.YELLOW) {
                                currentTarget = s;
                            }
                        }
                        break;
                    }*/
                    case OUTTAKE: {
                        if (gamepad1.a && !g.a) {
                            c1.setPosition(intake_positions[4]);
                            c2.setPosition(intake_positions[5]);

                            if (time == 0) {
                                time = getRuntime();
                            }
                        }

                        if (getRuntime() - time >= times[1] && time!=0) {

                            armMotor.setTargetPosition((int) intake_positions[0]);
                            liftMotor.setTargetPosition((int) intake_positions[1]);
                            pitchServo.setPosition(intake_positions[2]);
                            yawServo.setPosition(intake_positions[3]);


                            armMotor.setPower(intake_power[0]);
                            liftMotor.setPower(intake_power[1]);

                            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                            current_claw_state = CLAW_STATE.INTAKE;

                            time = 0;
                        }
                        break;
                    }
                    case HANG: {
                        armMotor.setTargetPosition(-940);
                        liftMotor.setTargetPosition(0);
                        pitchServo.setPosition(intake_positions[2]);
                        yawServo.setPosition(intake_positions[3]);
                        c1.setPosition(intake_positions[4]);
                        c2.setPosition(intake_positions[5]);

                        armMotor.setPower(0.2);
                        liftMotor.setPower(1);

                        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        if (gamepad1.dpad_down && !g.dpad_down) {
                            current_claw_state = CLAW_STATE.HANG_FINAL;
                        }
                        break;
                    }
                    
                    case HANG_FINAL: {
                        armMotor.setTargetPosition(-1200);
                        liftMotor.setTargetPosition(-10000);
                        pitchServo.setPosition(intake_positions[2]);
                        yawServo.setPosition(intake_positions[3]);
                        c1.setPosition(intake_positions[4]);
                        c2.setPosition(intake_positions[5]);

                        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        armMotor.setPower(0.8);
                        liftMotor.setPower(1);
                    }
                }

                if (gamepad1.dpad_up && !g.dpad_up) {
                    current_claw_state = CLAW_STATE.HANG;
                }

                telemetry.addData("switch", current_claw_state);
                telemetry.addData("lM", liftMotor.getCurrentPosition());
                telemetry.addData("aM", armMotor.getCurrentPosition());
                telemetry.update();
                g.copy(gamepad1);
            }
        }
    }
}
