package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.FeedForward;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.profile.MotionProfile;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.profile.OLD_PIDFController;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;

/**
 * A linear slide can extend and retract the wrist and claw attached to the robot's scoring
 * subsystem.
 */
@Config
public class LinearSlideWithProfile extends SubsystemBase {
    public static double SPOOL_DIAMETER = 2.025; // in inches (measured on 01/27 as 1.482")
    public static double TICKS_PER_REV = 537.7;
    public double GEARING = 1.0; // No gears

    public static double ACHIEVABLE_MAX_RPM_FRACTION = 1.0;

    public static double KP = 0.7;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double KG = 0.0;
    private double KV = 0;
    private double KA = 0;
    public static double KS = 0.0;
    public static double maxVelocity = 120;
    public static double maxAcceleration = 95;

    PIDCoefficients pidCoefficients = new PIDCoefficients(KP, KI, KD);

    // Constants for determining if the arm is at target
    public static double TOLERABLE_ERROR = 0.5; // inches
    public static int AT_TARGET_COUNT = 1;

    public static double MIN_EXTENSION_LENGTH = 0.25;
    public static double MAX_EXTENSION_LENGTH = 40;

    private final Motor slideMotor;
    private final Telemetry telemetry;
    private OLD_PIDFController pidfController;
    private MotionProfile profile;
    private double targetLength = 0;
    private int atTargetCount = 0;

    /**
     * Instantiates the linear slide for the robot with a static feed forward value of <code>KG</code>.
     *
     * @param hardwareMap Hardware Map
     * @param telemetry   Telemetry
     */
    public LinearSlideWithProfile(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, p->KG);
    }

    /**
     * Instantiates the linear slide for the robot with the specified feed forward.
     *
     * @param hardwareMap  Hardware Map
     * @param telemetry    Telemetry
     * @param feedForward  Feed Forward
     */
    public LinearSlideWithProfile(HardwareMap hardwareMap, Telemetry telemetry, FeedForward feedForward) {
        this.telemetry = telemetry;
        slideMotor = new Motor(hardwareMap, telemetry, "linearSlide");
        configMotor(slideMotor);
        pidfController = new OLD_PIDFController(pidCoefficients,KV,KA,KS, feedForward);
        pidfController.setOutputBounds(-1,1);
    }

    /**
     * Gets the target slide length in inches
     * Finds the value for the length
     *
     * @return target slide length in inches
     */
    public double getTargetLength() {
        return targetLength;
    }

    /**
     * sets the slide length from an external source and uses pid to make is
     *
     * @param length Length of desired slide position in inches.
     */
    public void setLength(double length) {
        double targetLength = Range.clip(length, MIN_EXTENSION_LENGTH, MAX_EXTENSION_LENGTH);
        if (this.targetLength != targetLength) {
            this.targetLength = targetLength;
            profile = new MotionProfile(maxAcceleration,maxVelocity,getCurrentLength(),targetLength);
            pidfController.reset();
            atTargetCount = 0;
        }
    }

    /**
     * Gets the slide length in inches
     * Finds the value for the length
     *
     * @return slide length in inches
     */
    public double getCurrentLength() {
        return slideMotor.getCurrentInches();
    }

    /**
     * Configures the motor used for the linear slide
     *
     * @param motor the motor to be configured
     */
    private void configMotor(Motor motor) {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setTicksPerRev(TICKS_PER_REV);
        motorConfigurationType.setGearing(GEARING);
        motorConfigurationType.setAchieveableMaxRPMFraction(ACHIEVABLE_MAX_RPM_FRACTION);
        motor.setMotorType(motorConfigurationType);
        motor.setMode(Motor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(Motor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(Motor.Direction.FORWARD);
        motor.setInchesPerRev(Math.PI * SPOOL_DIAMETER);
    }

    /**
     * sets the power for the pid controller
     */
    public void execute() {
        double currentLength = getCurrentLength();
        double profileTargetPosition = profile.calculatePosition();
        pidfController.setTargetPosition(profileTargetPosition);
        double power = pidfController.update(profile.getTimestamp(), currentLength);
        slideMotor.setPower(power);

        telemetry.addData("[LS] ProfileTargetPos", profileTargetPosition);
        telemetry.addData("[LS] Power", power);

        // Make sure the slide is at it's target for a number of consecutive loops. This is designed
        // to handle cases of "bounce" in the slide when moving to the target angle.
        double error = Math.abs(targetLength - currentLength);
        if (error <= TOLERABLE_ERROR) {
            atTargetCount++;
        } else {
            atTargetCount = 0;
        }
    }

    /**
     * checks if the length is within the tolerable error and if it is then the motor will stop
     */
    public boolean isAtTarget() {
        boolean atTarget = atTargetCount >= AT_TARGET_COUNT;
        telemetry.addData("[Slide] atTarget", atTarget);
        return atTarget;
    }

    /**
     * Returns a string representation of the linear slide.
     *
     * @return a string representation of the linear slide
     */
    @Override
    @NonNull
    public String toString() {
        return "LinearSlide{" +
                "target=" + targetLength +
                ", current=" + getCurrentLength() +
                "}";
    }
}
