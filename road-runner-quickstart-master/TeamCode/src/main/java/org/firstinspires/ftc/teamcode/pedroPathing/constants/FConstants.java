package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.KalmanFilterParameters;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
// This acts as a method of updating FollowerConstants without direct access to it.
public class FConstants { // This is how we change Follower Constants.
    private static double dP = 0.017;
    private static double dI = 0;
    private static double dD = 0.001;
    private static double dF = 0;

    private static double tP = 0.3;
    private static double tI = 0;
    private static double tD = 0.014;
    private static double tF = 0;

    private static double hP = 2.1;
    private static double hI = 0;
    private static double hD = 0.03;
    private static double hF = 0;

    public static double centri = 0.0017;


    public static CustomPIDFCoefficients hPIDF = new CustomPIDFCoefficients(hP, hI, hD, hF);
    public static CustomPIDFCoefficients tPIDF = new CustomPIDFCoefficients(tP, tI, tD, tF);
    public static CustomFilteredPIDFCoefficients dPIDF = new CustomFilteredPIDFCoefficients(dP, dI, dD,0.6, dF);

    static {
        // Select our localizer
        FollowerConstants.localizers = Localizers.PINPOINT;

        // We can change the value of any variable/constant of FollowerConstants.
        FollowerConstants.mass = 13.56241; // In kg

        FollowerConstants.leftFrontMotorName = "frontLeft";
        FollowerConstants.leftRearMotorName = "backLeft";
        FollowerConstants.rightFrontMotorName = "frontRight";
        FollowerConstants.rightRearMotorName = "backRight";
        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.xMovement = 75.74208651901873;
        FollowerConstants.yMovement = 54.888107876454555;

        FollowerConstants.translationalPIDFCoefficients = tPIDF;

        FollowerConstants.translationalPIDFFeedForward = 0.015;

        FollowerConstants.headingPIDFCoefficients = hPIDF;
        FollowerConstants.headingPIDFFeedForward = 0.01;

        FollowerConstants.drivePIDFCoefficients = dPIDF;

        FollowerConstants.drivePIDFFeedForward = 0.01;

        FollowerConstants.driveKalmanFilterParameters = new KalmanFilterParameters(
                6,
                1);

        FollowerConstants.centripetalScaling = centri;

        FollowerConstants.forwardZeroPowerAcceleration = -29.856985354926294;
        FollowerConstants.lateralZeroPowerAcceleration = -65.6483721920687;
        FollowerConstants.zeroPowerAccelerationMultiplier = 6;

        FollowerConstants.pathEndVelocityConstraint = 0.8;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndTimeoutConstraint = 100;
        FollowerConstants.APPROXIMATION_STEPS = 1000;
    }
}
