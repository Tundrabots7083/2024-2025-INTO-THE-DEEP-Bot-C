package org.firstinspires.ftc.teamcode.ftc7083;

import androidx.annotation.NonNull;

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
public class Robot {
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

        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        // Instantiate all the hardware on the robot
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        linearSlide = new LinearSlide(hardwareMap, telemetry, new LinearSlideFeedForward(arm, LinearSlide.KG));
        wrist = new Wrist(hardwareMap, telemetry);
        claw = new Claw(hardwareMap, telemetry);
        intakeAndScoringSubsystem = new IntakeAndScoringSubsystem(hardwareMap, telemetry);
        leftWebcam = new Webcam(hardwareMap, telemetry, Webcam.Location.LEFT, viewIds[0]);
        rightWebcam = new Webcam(hardwareMap, telemetry, Webcam.Location.RIGHT, viewIds[1]);
        limelight = new Limelight(hardwareMap, telemetry);
        colorSensor = new ColorSensor(hardwareMap, telemetry);
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        calibrateOTOS();

        webcams = Arrays.asList(leftWebcam, rightWebcam);
        localizer = new AprilTagAndOTOSLocalizer(webcams, otos);

        // Wait for all webcams to initialize
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

        this.telemetry.addLine("[Robot] initialized");
    }

    /**
     * Calibrate the SparkFun OTOS.
     */
    public void calibrateOTOS() {
        otos.resetTracking();
        otos.calibrateImu();
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
}
