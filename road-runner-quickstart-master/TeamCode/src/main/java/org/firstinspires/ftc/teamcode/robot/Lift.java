package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Lift {

    public static DcMotorEx  oM1;
    public DcMotorEx oM2;
    private Servo outtakeClaw, outtakePitch;
    private PIDFController controller;

    public static double target = 0.0;

    public static double sample_height = 4100;
    public static double specimen_height = 1800;

    public static double ascent_i = 600;
    public static double specimen_down = 1680;
    public static boolean pidfused = true;

    public static double clawOpen = 0.9;
    public static double clawClose = 0.63;
    public static double pitchDown = 0.53;
    public static double pitchUp = 0.21;

    public static double pitchPos = pitchDown;
    public static double clawPos = clawOpen;


    public Lift(HardwareMap hardwareMap) {
        controller = new PIDFController(0.01, 0.0001, 0.0003, 0.0001);
        oM1 = (DcMotorEx) hardwareMap.dcMotor.get("outtake1");
        oM2 = (DcMotorEx) hardwareMap.dcMotor.get("outtake2");
        oM2.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeClaw = hardwareMap.servo.get("outtakeClaw");
        outtakePitch = hardwareMap.servo.get("outtakePitch");
    }

    public void update() {
        if(pidfused) {
            oM1.setPower(controller.calculate(oM1.getCurrentPosition(), target));
            oM2.setPower(controller.calculate(oM1.getCurrentPosition(), target));
        }

        outtakePitch.setPosition(pitchPos);
        outtakeClaw.setPosition(clawPos);
    }

    public void clawClose() {
        clawPos = clawClose;
    }

    public static void clawOpen() {
        clawPos = clawOpen;
    }

    public static void pitchHome() {
            pitchPos = pitchDown;
    }

    public static void pitchDrop() {
        pitchPos = pitchUp;
    }

    public static void sample() {
        target = 4000;
    }

    public void home() {
        target = 10;
    }

    public Action clawDrop() {
        return new ClawDrop();
    }

    public Action ascent() {
        return new Ascent();
    }

    public static Action clawGrab() {
        return new ClawGrab();
    }

    public Action clawReturn() {
        return new ClawReturn();
    }

    public Action liftDown() {
        return new LiftDown();
    }

    public Action liftUp() {
        return new LiftUp();
    }

    public Action specimenDown() {
        return new SpecimenDown();
    }

    public Action specimenHeight() {
        return new SpecimenHeight();
    }
    public Action specimenRelease() {
        return new SpecimenRelease();
    }



    public static class Ascent implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init) {
                Lift.target = 700;
                init = true;
            }

            return oM1.getCurrentPosition() < 650;
        }

        public Ascent( ) {

        }


    }

    public static class ClawDrop implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init) {
                pitchDrop();
                clawOpen();
                init = true;
            }

            return false;
        }

        public ClawDrop() {

        }


    }

    public static class ClawGrab implements Action {
        private boolean init = false;


        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init) {
                pitchHome();
                clawOpen();
                init = true;
            }

            return false;
        }

        public ClawGrab() {

        }


    }

    public class ClawReturn implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init) {
                pitchHome();
                clawOpen();
                init = true;
            }

            return false;
        }

        public ClawReturn() {

        }


    }

    public class LiftDown implements Action {
        private boolean init = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init) {
                home();
                init = true;
            }

            if(oM1.getCurrentPosition() > 200) {
                update();
                return true;
            }

            return false;
        }

        public LiftDown() {

        }


    }

    public static class LiftUp implements Action {
        private boolean init = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init) {
                sample();
                init = true;
            }

            return oM1.getCurrentPosition() < 2800;
        }

        public LiftUp() {

        }


    }

    public static class SpecimenDown implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init){
                Lift.target = Lift.specimen_down;
            }

            if(oM1.getCurrentPosition() <= Lift.specimen_down - 100) {
                return false;
            }

            return true;
        }

        public SpecimenDown() {

        }

    }

    public static class SpecimenHeight implements Action {
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init){
                Lift.target = Lift.specimen_height;
            }

            if(oM1.getCurrentPosition() >= Lift.specimen_height - 100) {
                return false;
            }

            return true;
        }

        public SpecimenHeight() {

        }

    }

    public static class SpecimenRelease implements Action {
        private boolean init = false;
        private double t = -1;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init){
                clawOpen();
            }

            return true;
        }

        public SpecimenRelease() {

        }


    }

    public static class PitchUp implements Action {
        private boolean init = false;
        private double t = -1;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init){
                pitchDrop();
            }

            return true;
        }

        public PitchUp() {

        }


    }

    public static class PitchDown implements Action {
        private boolean init = false;
        private double t = -1;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init){
                pitchHome();
            }

            return true;
        }

        public PitchDown() {

        }


    }
}


