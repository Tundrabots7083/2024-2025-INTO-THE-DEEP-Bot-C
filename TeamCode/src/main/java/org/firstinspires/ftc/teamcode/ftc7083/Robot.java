package org.firstinspires.ftc.teamcode.ftc7083;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.SparkFunOTOS;
import org.firstinspires.ftc.teamcode.ftc7083.localization.AprilTagAndOTOSLocalizer;
import org.firstinspires.ftc.teamcode.ftc7083.localization.Localizer;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Arm;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Claw;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Limelight;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Webcam;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Wrist;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Arrays;
import java.util.List;

/**
 * The Robot. This is implemented as a singleton, meaning there is one robot instance that exists.
 */
@Config
public class Robot {
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

    // RR localizer note: These units are inches and radians.
    public static double SPARKFUN_OTOS_OFFSET_X = 0.0;
    public static double SPARKFUN_OTOS_OFFSET_Y = 0.0;
    public static double SPARKFUN_OTOS_HEADING_IN_DEGREES = 0.0;

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
    public static double SPARKFUN_LINEAR_SCALAR = 1.0;
    public static double SPARKFUN_ANGULAR_SCALAR = 1.0;

    private static Robot robot = null;

    public final Telemetry telemetry;

    // Subsystems and hardware
    public final MecanumDrive mecanumDrive;
    public final IntakeAndScoringSubsystem intakeAndScoringSubsystem;
    public final Webcam leftWebcam;
    public final Webcam rightWebcam;
    public final Arm arm;
    public final LinearSlide linearSlide;
    public final Wrist wrist;
    public final Claw claw;
    public final Limelight limelight;
    public final SparkFunOTOS otos;
    public final ColorSensor colorSensor;

    public final List<Webcam> webcams;

    // Road Runner localization
    public final Localizer localizer;

    // All lynx module hubs
    public final List<LynxModule> allHubs;

    /**
     * Creates a new instance of the robot.
     *
     * @param hardwareMap    hardware map for the robot.
     * @param telemetry      telemetry class for displaying data.
     * @param opModeType     the type of opmode the robot is being used for
     * @param armFeedForward feed forward value for the arm
     */
    private Robot(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry, OpModeType opModeType, double armFeedForward) {
        robot = this;
        this.telemetry = telemetry;

        // Enable bulk reads. This is almost always the "correct" answer, and can speed up loop
        // times. We will be managing the bulk read caches manually, which requires each OpMode
        // to clear the cache at the start of each loop.
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        // Instantiate all the hardware on the robot
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry, armFeedForward);
        linearSlide = new LinearSlide(hardwareMap, telemetry);
        wrist = new Wrist(hardwareMap, telemetry);
        claw = new Claw(hardwareMap, telemetry);
        intakeAndScoringSubsystem = new IntakeAndScoringSubsystem(hardwareMap, telemetry);
        leftWebcam = new Webcam(hardwareMap, telemetry, Webcam.Location.LEFT, viewIds[0]);
        rightWebcam = new Webcam(hardwareMap, telemetry, Webcam.Location.RIGHT, viewIds[1]);
        limelight = new Limelight(hardwareMap, telemetry);
        colorSensor = new ColorSensor(hardwareMap, telemetry);
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        webcams = Arrays.asList(leftWebcam, rightWebcam);
        localizer = new AprilTagAndOTOSLocalizer(webcams, otos);

        this.telemetry.addLine("[Robot] initialized");
        this.telemetry.update();
    }

    /**
     * Initializes the hardware mechanisms for the robot. This creates the singleton that is retrieved
     * using the <code>getInstance</code> method.
     *
     * @param hardwareMap hardware map for the robot.
     * @param telemetry   telemetry class for displaying data.
     * @return the robot instance
     */
    public static Robot init(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        return init(hardwareMap, telemetry, OpModeType.TELEOP);
    }

    /**
     * Initializes the hardware mechanisms for the robot. This creates the singleton that is retrieved
     * using the <code>getInstance</code> method.
     *
     * @param hardwareMap hardware map for the robot.
     * @param telemetry   telemetry class for displaying data.
     * @param opModeType  the type of opmode the robot is being used for
     * @return the robot instance
     */
    public static Robot init(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry, OpModeType opModeType) {
        return init(hardwareMap, telemetry, OpModeType.TELEOP, 0.0);
    }

    /**
     * Initializes the hardware mechanisms for the robot. This creates the singleton that is retrieved
     * using the <code>getInstance</code> method.
     *
     * @param hardwareMap hardware map for the robot.
     * @param telemetry   telemetry class for displaying data.
     * @param opModeType  the type of opmode the robot is being used for
     * @return the robot instance
     */
    public static Robot init(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry, OpModeType opModeType, double armFeedForward) {
        robot = new Robot(hardwareMap, telemetry, opModeType, armFeedForward);
        return robot;
    }

    /**
     * Gets the singleton instance of the robot.
     */
    public static Robot getInstance() {
        return robot;
    }

    /**
     * Gets a string representation of the robot.
     *
     * @return a string representation of the robot
     */
    @NonNull
    @Override
    public String toString() {
        return "Robot{" +
                "mecanumDrive=" + mecanumDrive +
                ", webcam=" + leftWebcam +
                '}';
    }

    // Enum to specify opmode type
    public enum OpModeType {
        /**
         * Driver controlled OpMode
         */
        TELEOP,
        /**
         * Autonomous OpMode
         */
        AUTO
    }
}
