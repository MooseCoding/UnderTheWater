package org.firstinspires.ftc.teamcode.old;

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

import org.firstinspires.ftc.teamcode.robot.vision.Sample;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;


@Photon
@TeleOp
@Disabled
public class USEME extends LinearOpMode {
    private final Scalar lowerBlue = new Scalar(100, 150, 50);
    private final Scalar upperBlue = new Scalar(130,255,255);
    private final Scalar lowerRed1 = new Scalar(0,150,50);
    private final Scalar upperRed1 = new Scalar(10,255,255);
    private final Scalar lowerRed2 = new Scalar(170,150,50);
    private final Scalar upperRed2 = new Scalar(180,255,255);
    private final Scalar lowerYellow = new Scalar(20,150,50);
    private final Scalar upperYellow = new Scalar(30,255,255);

    private DcMotorEx fL, fR, bL, bR;
    private DcMotorEx iM;
    private Servo c1, c2, yawServo, pitchServo;

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

    private double[] homes = {100,0,  0,0,0}; //intake, lift, gps, gc1, gc2,
    private double[] homes_power = {0.7,1};
    private double[] intake_positions = {300,0.35,0.39,  0.3, 0.8}; // in order: intake -> pitch servo -> yaw servo -> c1 -> c2
    private double[] intake_power = {0.8};

    private double[] grab_positions = {0.2,0.4,  0.8,0.3}; //  pitch -> yaw -> c1 -> c2


    private OpenCvCamera camera;
    private int cID;
    private int iMPOS = 0;

    public static ArrayList<Sample> samples = new ArrayList<>();
    public void ExtendIntake(int position, double power) {
        iM.setTargetPosition(position);
        iM.setPower(power);
        iM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

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

            iM = (DcMotorEx) hardwareMap.dcMotor.get("intake");

            c1 = hardwareMap.servo.get("leftServo");fL = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
            bL = (DcMotorEx) hardwareMap.dcMotor.get("backLeft");
            fR = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
            bR = (DcMotorEx) hardwareMap.dcMotor.get("backRight");
            c2 = hardwareMap.servo.get("rightServo");
            yawServo = hardwareMap.servo.get("yawServo");
            pitchServo = hardwareMap.servo.get("pitchServo");

            current_claw_state = CLAW_STATE.STARTING;

            fR.setDirection(DcMotorSimple.Direction.REVERSE);
            bL.setDirection(DcMotorSimple.Direction.REVERSE);
            bR.setDirection(DcMotorSimple.Direction.REVERSE);
            iM.setDirection(DcMotorSimple.Direction.REVERSE);

            iM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
                            iM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            iM.setPower(gamepad1.right_trigger);
                        }
                        else if(gamepad1.left_trigger > 0) {
                            iM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            iM.setPower(-gamepad1.left_trigger);
                        }
                        else {
                            iM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            iM.setPower(0);
                        }

                        if(gamepad1.cross && t == -1) {
                            c1.setPosition(grab_positions[2]);
                            c2.setPosition(grab_positions[3]);

                            t = getRuntime();
                        }

                        if (t != -1 && getRuntime() - t > 1) {
                            pitchServo.setPosition(grab_positions[0]);
                            t = -1;

                            iM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            ExtendIntake((int) homes[0], homes_power[0]);
                            current_claw_state = CLAW_STATE.GRAB;
                        }
                        break;
                    }
                    case GRAB: {
                            if (t == -1) {
                                if (gamepad1.cross) {

                                        pitchServo.setPosition(intake_positions[1]);

                                        t = getRuntime();

                                }

                                if (gamepad1.right_trigger > 0 && t == -1) {
                                    iM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                    iM.setPower(gamepad1.right_trigger);
                                }
                                else if(gamepad1.left_trigger > 0) {
                                    iM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                    iM.setPower(-gamepad1.left_trigger);
                                }
                                else {
                                    iM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                    iM.setPower(0);
                                }
                            }
                                    if(t != -1 && getRuntime() - t >= 0.5) {
                                        pitchServo.setPosition(intake_positions[1]);
                                        yawServo.setPosition(intake_positions[2]);
                                        c1.setPosition(intake_positions[3]);
                                        c2.setPosition(intake_positions[4]);
                                        iM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                        ExtendIntake((int)homes[0], homes_power[0]);
                                        current_claw_state = CLAW_STATE.STARTING;
                                        t = -1;
                                    }
                        break;
                    }
                }

                if (gamepad1.dpad_up){
                    ExtendIntake(0, 0.8);
                }

                telemetry.addData("iM", iM.getCurrentPosition());
                telemetry.update();
                g.copy(gamepad1);
            }
        }
    }
}
