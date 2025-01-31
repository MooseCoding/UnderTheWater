package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.constants.PinpointConstants;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// This acts as a method of updating ThreeWheelConstants without direct access to it.
@Config
public class LConstants { // This is how we change ThreeWheelConstants.
    static public double forwardY = 4.5;
    static public double strafeX = -7.5;

    static {

        PinpointConstants.forwardY = -4.5;
        PinpointConstants.strafeX = -7.5;
        PinpointConstants.distanceUnit = DistanceUnit.INCH;
        PinpointConstants.hardwareMapName = "pp";
        PinpointConstants.useYawScalar = false;
        PinpointConstants.yawScalar = 1.0;
        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    }
}
