package org.firstinspires.ftc.teamcode.ftc7083;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive.SparkFunParams;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive.Params;
import org.firstinspires.ftc.teamcode.ftc7083.localization.AprilTagAndOTOSLocalizer;
import org.firstinspires.ftc.teamcode.ftc7083.localization.Localizer;
import org.firstinspires.ftc.teamcode.ftc7083.localization.SparkFunOTOSLocalizer;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Arm;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.ArmWithProfile;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Claw;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.GlobalShutterCamera;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Limelight;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.LinearSlideWithProfile;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Webcam;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.feedback.LinearSlideFeedForward;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Wrist;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Arrays;
import java.util.List;

/**
 * The Robot. This is implemented as a singleton, meaning there is one robot instance that exists.
 *
 * <ul>
 *     <li>
 *         Drive Chassis
 *         <ul>
 *             <li>
 *                 Left Front Motor: Control Hub 1
 *             </li>
 *             <li>
 *                 Left Rear Motor: Control Hub 3
 *             </li>
 *             <li>
 *                 Right Front Motor: Control Hub 0
 *             </li>
 *             <li>
 *                 Right Rear Motor: Control Hub 2
 *             </li>
 *         </ul>
 *     </li>
 *     <li>
 *         Arm
 *         <ul>
 *             <li>
 *                 Motor: Expansion Hub Port 2
 *             </li>
 *             <li>
 *                 Encoder: Expansion Hub Port 2
 *             </li>
 *         </ul>
 *     </li>
 *     <li>
 *         Linear Slide
 *         <ul>
 *             <li>
 *                 Motor: Expansion Hub Port 0
 *             </li>
 *             <li>
 *                 Encoder: Expansion Hub Port 0
 *             </li>
 *         </ul>
 *     </li>
 *     <li>
 *         Wrist
 *         <ul>
 *             Pitch
 *             <ul>
 *                 Servo: Expansion Hub Port 5
 *             </ul>
 *         </li>
 *         <li>
 *             Roll
 *             <ul>
 *                 Servo: Expansion Hub Port 3
 *             </ul>
 *         </ul>
 *     </li>
 *     <li>
 *         Claw
 *         <ul>
 *             Servo: Expansion Hub Port 1
 *         </ul>
 *     </li>
 *     <li>
 *         Color Sensor
 *         <ul>
 *             Control Hub I2C Port 1
 *         </ul>
 *     </li>
 *     <li>
 *         SparkFun OTOS
 *         <ul>
 *             Control Hub I2C Port 2
 *         </ul>
 *     </li>
 * </ul>
 */
@Config
public class Robot {
    public static boolean USE_WEBCAMS = false;
    public static boolean USE_SPARKFUN_OTOS_CORRECTED = true;

    private static Robot robot = null;

    public static SampleIntakeColor INTAKE_COLOR = SampleIntakeColor.YELLOW;

    public final Telemetry telemetry;

    // Subsystems and hardware
    public final MecanumDrive mecanumDrive;
    public final IntakeAndScoringSubsystem intakeAndScoringSubsystem;
    public Webcam leftWebcam;
    public Webcam rightWebcam;
    public final ArmWithProfile arm;
    public final LinearSlideWithProfile linearSlide;
    public final Wrist wrist;
    public final Claw claw;
    public Limelight limelight;
    public final SparkFunOTOS otos;
    public GlobalShutterCamera globalShutterCamera;
    //public final ColorSensor colorSensor;

    public List<Webcam> webcams;

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
     */
    private Robot(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry, OpModeType opModeType) {
        robot = this;
        this.telemetry = telemetry;

        // Enable bulk reads. This is almost always the "correct" answer, and can speed up loop
        // times. We will be managing the bulk read caches manually, which requires each OpMode
        // to clear the cache at the start of each loop.
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        // Instantiate all the hardware on the robot
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        arm = new ArmWithProfile(hardwareMap, telemetry);
        linearSlide = new LinearSlideWithProfile(hardwareMap, telemetry, new LinearSlideFeedForward(arm, LinearSlideWithProfile.KG));
        wrist = new Wrist(hardwareMap, telemetry);
        claw = new Claw(hardwareMap, telemetry);
        intakeAndScoringSubsystem = new IntakeAndScoringSubsystem(hardwareMap, telemetry);
        globalShutterCamera = new GlobalShutterCamera(hardwareMap, telemetry);
        limelight = new Limelight(hardwareMap, telemetry);
        //colorSensor = new ColorSensor(hardwareMap, telemetry);

        if (USE_SPARKFUN_OTOS_CORRECTED) {
            otos = hardwareMap.get(SparkFunOTOSCorrected.class, "otos");
        } else {
            otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        }
        telemetry.addLine("Up to OTOS");
        telemetry.update();
        calibrateOTOS();

        // Use a localizer with the OTOS and webcams, or just the OTOS
        /*if (USE_WEBCAMS) {
            int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);


            leftWebcam = new Webcam(hardwareMap, telemetry, Webcam.Location.LEFT, viewIds[0]);
            rightWebcam = new Webcam(hardwareMap, telemetry, Webcam.Location.RIGHT, viewIds[1]);
            webcams = Arrays.asList(leftWebcam, rightWebcam);
            localizer = new AprilTagAndOTOSLocalizer(webcams, otos);

            //Wait for all webcams to initialize
            boolean webcamsInitialized = false;
            while (!webcamsInitialized) {
                webcamsInitialized = true;
                for (Webcam webcam : webcams) {
                    if (!webcam.webcamInitialized()) {
                        webcamsInitialized = false;
                        break;
                    }
                }
            }
        } else {

        }*/

        localizer = new SparkFunOTOSLocalizer(otos);
        this.telemetry.addLine("[Robot] initialized");
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
        robot = new Robot(hardwareMap, telemetry, opModeType);
        return robot;
    }

    /**
     * Gets the singleton instance of the robot.
     */
    public static Robot getInstance() {
        return robot;
    }

    /**
     * Calibrate the SparkFun OTOS.
     */
    private void calibrateOTOS() {
        telemetry.addLine("OTOS calibration beginning!");

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        otos.resetTracking();

        // RR localizer note:
        // don't change the units, it will stop Dashboard field view from working properly
        // and might cause various other issues
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        // Set the offset for the OTOS
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(SparkFunParams.MOUNTING_OFFSET_X,
                SparkFunParams.MOUNTING_OFFSET_Y,
                Math.toRadians(SparkFunParams.MOUNTING_HEADING_IN_DEGREES));
        otos.setOffset(offset);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your programs. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total

        // RR localizer note: It is technically possible to change the number of samples to slightly reduce init times,
        // however, I found that it caused pretty severe heading drift.
        // Also, if you're careful to always wait more than 612ms in init, you could technically disable waitUntilDone;
        // this would allow your OpMode code to run while the calibration occurs.
        // However, that may cause other issues.
        // In the future I hope to do that by default and just add a check in updatePoseEstimate for it
        otos.calibrateImu(SparkFunParams.NUM_IMU_CALIBRATION_SAMPLES, true);

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
        otos.setLinearScalar(SparkFunParams.LINEAR_SCALAR);
        otos.setAngularScalar(SparkFunParams.ANGULAR_SCALAR);

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(Params.INITIAL_POS_X,
                                                                      Params.INITIAL_POS_Y,
                                                                      Math.toRadians(Params.INITIAL_POS_HEADING));
        otos.setPosition(currentPosition);

        telemetry.addLine("OTOS calibration complete!");
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
                ", arm=" + arm +
                ", linearSlide=" + linearSlide +
                ", wrist=" + wrist +
                ", claw=" + claw +
                ", otos=" + otos +
                //", leftWebcam=" + leftWebcam +
                //", rightWebcam=" + rightWebcam +
                //", limelight=" + limelight +
                //", colorSensor=" + colorSensor +
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

    // Enum to specify color for intake
    public enum SampleIntakeColor {
        RED,
        BLUE,
        YELLOW
    }
}
