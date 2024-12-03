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

    public DcMotorEx  oM1, oM2;
    private Servo outtakeClaw, outtakePitch;
    private PIDFController controller;

    public static double target = 0.0;
    public static double specimen_height = 2000;
    public static double specimen_down = 1600;
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
        oM2.setDirection(DcMotorSimple.Direction.REVERSE);
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

    public void clawOpen() {
        clawPos = clawOpen;
    }

    public void pitchHome() {
            pitchPos = pitchDown;
    }

    public void pitchDrop() {
        pitchPos = pitchUp;
    }

    public void sample() {
        target = 4000;
    }

    public void home() {
        target = 10;
    }


    public class Ascent implements Action {
        private boolean init = false;
        private Lift claw;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init) {
                Lift.target = 1000;
                init = true;
            }

            return claw.oM1.getCurrentPosition() < 800;
        }

        public Ascent(Lift li) {
            claw = li;
        }

        public Action ascent(Lift li) {
            return new Ascent(li);
        }
    }

    public class ClawDrop implements Action {
        private boolean init = false;
        private Lift claw;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init) {
                claw.pitchDrop();
                claw.clawOpen();
                init = true;
            }

            return false;
        }

        public ClawDrop(Lift li) {
            claw = li;
        }

        public Action clawDrop(Lift li) {
            return new ClawDrop(li);
        }
    }

    public class ClawGrab implements Action {
        private boolean init = false;
        private Lift claw;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init) {
                claw.pitchHome();
                claw.clawOpen();
                init = true;
            }

            return false;
        }

        public ClawGrab(Lift li) {
            claw = li;
        }

        public Action clawGrab(Lift li) {
            return new ClawGrab(li);
        }
    }

    public class ClawReturn implements Action {
        private boolean init = false;
        private Lift claw;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init) {
                claw.pitchHome();
                claw.clawOpen();
                init = true;
            }

            return false;
        }

        public ClawReturn(Lift li) {
            claw = li;
        }

        public Action clawReturn(Lift li) {
            return new ClawReturn(li);
        }
    }

    public class LiftDown implements Action {
        private boolean init = false;
        private Lift lift;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init) {
                lift.home();
                init = true;
            }

            if(lift.oM1.getCurrentPosition() > 200) {
                lift.update();
                return true;
            }

            return false;
        }

        public LiftDown(Lift li) {
            lift = li;
        }

        public Action liftDown(Lift li) {
            return new LiftDown(li);
        }
    }

    public class LiftUp implements Action {
        private boolean init = false;
        private Lift lift;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init) {
                lift.sample();
                init = true;
            }

            if(lift.oM1.getCurrentPosition() < 2800) {
                return true;
            }

            return false;
        }

        public LiftUp(Lift li) {
            lift = li;
        }

        public Action liftUp(Lift li) {
            return new LiftUp(li);
        }
    }

    public class SpecimenDown implements Action {
        private Lift i;
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init){
                Lift.target = Lift.specimen_down;
            }

            if(i.oM1.getCurrentPosition() <= Lift.specimen_down - 100) {
                return false;
            }

            return true;
        }

        public SpecimenDown(Lift li) {
            i = li;
        }

        public Action specimenDown(Lift li) {
            return new SpecimenDown(li);
        }
    }

    public class SpecimenHeight implements Action {
        private Lift i;
        private boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!init){
                Lift.target = Lift.specimen_height;
            }

            if(i.oM1.getCurrentPosition() >= Lift.specimen_height - 100) {
                return false;
            }

            return true;
        }

        public SpecimenHeight(Lift li) {
            i = li;
        }

        public Action specimenHeight(Lift li) {
            return new SpecimenHeight(li);
        }
    }
}


