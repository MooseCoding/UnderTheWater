package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.vision.Color;
import org.firstinspires.ftc.teamcode.vision.Sample;
import org.firstinspires.ftc.teamcode.vision.robotPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Photon
@TeleOp
public class FindingValues extends LinearOpMode {
    private Gamepad g = new Gamepad();
    private DcMotorEx liftMotor, armMotor;
    private Servo c1, c2, yawServo, pitchServo;
    private OpenCvCamera camera;
    private int cID;

    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStopRequested()) {
            armMotor = (DcMotorEx) hardwareMap.dcMotor.get("armMotor");
            liftMotor = (DcMotorEx) hardwareMap.dcMotor.get("liftMotor");

            c1 = hardwareMap.servo.get("leftServo");
            c2 = hardwareMap.servo.get("rightServo");
            yawServo = hardwareMap.servo.get("yawServo");
            pitchServo = hardwareMap.servo.get("pitchServo");

            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //cID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()); // camera id
            //camera = OpenCvCameraFactory.getInstance().createWebcam( hardwareMap.get(WebcamName.class, "cam"), cID); // camera object
            //camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);

            /*camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
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
            });*/

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

                if(gamepad1.right_trigger > 0) {
                    armMotor.setTargetPosition(-600);
                    liftMotor.setTargetPosition(6000);
                    armMotor.setPower(0.5);
                    liftMotor.setPower(1);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                else if (gamepad1.left_trigger > 0) {
                    liftMotor.setPower(-1);
                }
                else {
                     liftMotor.setPower(0);
                }

                if (gamepad1.left_bumper) {
                    armMotor.setPower(-0.7);
                }
                else if(gamepad1.right_bumper) {
                    armMotor.setPower(0.7);
                }
                else {
                    armMotor.setPower(0);
                }

                if(gamepad1.left_stick_button && !g.left_stick_button) {
                    liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                if (gamepad1.right_stick_button && !g.right_stick_button) {
                    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                telemetry.addData("aM", armMotor.getCurrentPosition());
                telemetry.addData("lM", liftMotor.getCurrentPosition());
                telemetry.update();

                g.copy(gamepad1);
            }
        }
    }
}
