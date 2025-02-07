package org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * Parameters originally embedded in RoadRunner's MecanumDrive, split out here for ease of finding
 * and configuring.
 */
@Config
public class Params {
    // IMU orientation
    public static RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

    // Initial robot position. This can be overridden using the setPosition() method.
    public static int INITIAL_POS_X = 0;
    public static int INITIAL_POS_Y = 0;
    public static int INITIAL_POS_HEADING = 90;

    // drive model parameters
    public static double inPerTick = 1; // If you're using OTOS/Pinpoint leave this at 1 (all values will be in inches, 1 tick = 1 inch)
    public static double lateralInPerTick = 4.1345; // // Tune this with LateralRampLogger (even if you use OTOS/Pinpoint)
    public static double trackWidthTicks = 1.6684;

    // feedforward parameters (in tick units)
    public static double kS = 0.8586;
    public static double kV = 0.1309;
    public static double kA = 0.001; // Initial value; increase by factor of 10 to find valid value

    // path profile parameters (in inches)
    public static double maxWheelVel = 50;
    public static double minProfileAccel = -30;
    public static double maxProfileAccel = 50;

    // turn profile parameters (in radians)
    public static double maxAngVel = Math.PI; // shared with path
    public static double maxAngAccel = Math.PI;

    // path controller gains
    public static double axialGain = 8.0;
    public static double lateralGain = 3.75;
    public static double headingGain = -0.35; // shared with turn

    public static double axialVelGain = 0.0;
    public static double lateralVelGain = 0.0;
    public static double headingVelGain = 0.0; // shared with turn
}
