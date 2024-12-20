package org.firstinspires.ftc.teamcode.old;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.vision.Sample;

import java.util.ArrayList;

@Photon
@TeleOp
@Disabled
public class NewTeleOP extends OpMode {
    private Telemetry tel;

    // Robot motors & servos
    private DcMotorEx fL, fR, bL, bR;

    // Subsystems
    private Lift l; // 
    private Intake i;
    private final Robot robot = new Robot();
    @Override
    public void init() {
        fL = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
        bL = (DcMotorEx) hardwareMap.dcMotor.get("backLeft");
        fR = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
        bR = (DcMotorEx) hardwareMap.dcMotor.get("backRight");

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        l = new Lift(hardwareMap);
        i = new Intake(hardwareMap);

        Intake.pidused = false;
    }

    private ArrayList<Sample> samples = new ArrayList<>();

    @Override
    public void loop() {
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

        //robot.loop_func(tel, gamepad1, samples, getRuntime());

        if(gamepad1.right_trigger > 0 && !Intake.pidused) {
            i.iM.setPower(0.3);
        }
        else if(gamepad1.left_trigger > 0 && !Intake.pidused) {
            i.iM.setPower(-0.3);
        }
        else if(!Intake.pidused){i.iM.setPower(0);}

        if(gamepad1.triangle && !Intake.pidused) {
            i.clawOpen();
            i.pitchDown();
        }

        if(gamepad1.circle && !Intake.pidused) {
            i.clawClose();
        }

        if(gamepad1.cross && !Intake.pidused) {
            i.pitchUp();
        }

        if(gamepad1.left_stick_button && !Intake.pidused) {
            Intake.pidused = true;
            Intake.target = 100;
        }

        if(gamepad1.dpad_up) {
            l.clawClose();
        }

        if(gamepad1.dpad_right) {
            i.clawPartial();
        }

        if(gamepad1.dpad_down) {
            Lift.pidfused = true;
            Lift.target = 3800;
        }

        if(gamepad1.dpad_left) {
            l.pitchDrop();
        }

        if(gamepad1.left_bumper) {
            l.clawOpen();
        }

        if(gamepad1.right_bumper) {
            l.clawClose();
        }

        if(gamepad1.triangle && Intake.pidused) {
            l.pitchHome();
        }

        if(gamepad1.circle && Intake.pidused) {
            Lift.target = 1000;
        }

        if(gamepad1.cross && Intake.pidused) {
            Lift.target = 0;
        }

        if(gamepad1.square && Intake.pidused) {
            Lift.pidfused = false;
            Intake.pidused = false;
            l.clawOpen();
            i.clawClose();
        }

        l.update();
        i.update();

        telemetry.addData("intake pid", Intake.pidused);
        telemetry.update();
    }
}
