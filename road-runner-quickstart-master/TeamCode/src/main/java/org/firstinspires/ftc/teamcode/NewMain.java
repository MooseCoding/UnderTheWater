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
public class NewMain extends LinearOpMode {
    private DcMotorEx fL, fR, bL, bR;
    private DcMotorEx iM, oM1, oM2;
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

            /*
            c1 = hardwareMap.crservo.get("leftServo");
            c2 = hardwareMap.crservo.get("rightServo");
            yawServo = hardwareMap.crservo.get("yawServo");
            pitchServo = hardwareMap.crservo.get("pitchServo");
             */
            current_claw_state = CLAW_STATE.STARTING;

            bR.setDirection(DcMotorSimple.Direction.REVERSE);
            fR.setDirection(DcMotorSimple.Direction.REVERSE);
            iM.setDirection(DcMotorSimple.Direction.REVERSE);
            oM2.setDirection(DcMotorSimple.Direction.REVERSE);
            iM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            oM1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            oM2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            /*
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
                }
            });*/

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

                        break;
                    }
                    case INTAKE: {

                        break;
                    }
                    case GRAB: {

                        break;
                    }
                    case OUTTAKE: {

                    }
                }

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

                /*
                if (gamepad1.right_trigger > 0){
                    oM1.setTargetPosition(2800);
                    oM2.setTargetPosition(2800);
                    oM1.setPower(1);
                    oM2.setPower(1);
                    oM2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    oM1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                else if (gamepad1.left_trigger > 0) {
                    oM1.setTargetPosition(0);
                    oM2.setTargetPosition(0);
                    oM1.setPower(1);
                    oM2.setPower(1);
                    oM2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    oM1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }*/

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
