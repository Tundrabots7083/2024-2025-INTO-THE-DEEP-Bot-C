package org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SparkFunParams {

    // RR localizer note: It is technically possible to change the number of samples to slightly reduce init times,
    // however, I found that it caused pretty severe heading drift.
    // Also, if you're careful to always wait more than 612ms in init, you could technically disable waitUntilDone;
    // this would allow your OpMode code to run while the calibration occurs.
    // However, that may cause other issues.
    // In the future I hope to do that by default and just add a check in updatePoseEstimate for it
    public static int NUM_IMU_CALIBRATION_SAMPLES = 255;

    // Assuming you've mounted your sensor to a robot and it's not centered,
    // you can specify the offset for the sensor relative to the center of the
    // robot. The units default to inches and degrees, but if you want to use
    // different units, specify them before setting the offset! Note that as of
    // firmware version 1.0, these values will be lost after a power cycle, so
    // you will need to set them each time you power up the sensor. For example, if
    // the sensor is mounted 5 inches to the left (negative X) and 10 inches
    // forward (positive Y) of the center of the robot, and mounted 90 degrees
    // clockwise (negative rotation) from the robot's orientation, the offset
    // would be {-5, 10, -90}. These can be any value, even the angle can be
    // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
    public static double MOUNTING_OFFSET_X = 0;
    public static double MOUNTING_OFFSET_Y = 0;
    public static double MOUNTING_HEADING_IN_DEGREES = 89.396;

    // Here we can set the linear and angular scalars, which can compensate for
    // scaling issues with the sensor measurements. Note that as of firmware
    // version 1.0, these values will be lost after a power cycle, so you will
    // need to set them each time you power up the sensor. They can be any value
    // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
    // first set both scalars to 1.0, then calibrate the angular scalar, then
    // the linear scalar. To calibrate the angular scalar, spin the robot by
    // multiple rotations (eg. 10) to get a precise error, then set the scalar
    // to the inverse of the error. Remember that the angle wraps from -180 to
    // 180 degrees, so for example, if after 10 rotations counterclockwise
    // (positive rotation), the sensor reports -15 degrees, the required scalar
    // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
    // robot a known distance and measure the error; do this multiple times at
    // multiple speeds to get an average, then set the linear scalar to the
    // inverse of the error. For example, if you move the robot 100 inches and
    // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
    public static double LINEAR_SCALAR = 1.209;
    public static double ANGULAR_SCALAR = 0.998;
}
