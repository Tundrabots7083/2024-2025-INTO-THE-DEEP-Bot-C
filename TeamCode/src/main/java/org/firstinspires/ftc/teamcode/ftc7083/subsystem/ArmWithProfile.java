package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionEx;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionExBase;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.profile.MotionProfile;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.profile.OLD_PIDFController;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.feedback.ArmFeedForwardWithProfile;

/**
 * An Arm is used to move the scoring subsystem in a circular arc, allowing the robot to both
 * pickup and score sample and specimens.
 */
@Config
public class ArmWithProfile extends SubsystemBase {
    // PID tuning values
    public static double KP = 0.15;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double KG = 0.1;
    private final double KV = 0;
    private final double KA = 0;
    public static double KS = 0.0;
    public static double maxVelocity = 200;
    public static double maxAcceleration = 250;

    PIDCoefficients pidCoefficients = new PIDCoefficients(KP, KI, KD);

    // Constants for determining if the arm is at target
    public static double TOLERABLE_ERROR = 2.5; // In degrees
    public static int AT_TARGET_COUNT = 1;

    public static double GEARING = 2.45;
    public static double START_ANGLE = -36.0;
    public static double ACHIEVABLE_MAX_RPM_FRACTION = 1.0;
    public static double TICKS_PER_REV = 1993.6; // GoBuilda ticks per rev
    public static double MIN_ANGLE = -36.0;
    public static double MAX_ANGLE = 100.0;

    private final Motor shoulderMotor;
    private final Telemetry telemetry;
    private OLD_PIDFController pidfController;
    private MotionProfile profile;
    private double targetAngle = START_ANGLE;
    private int atTargetCount = 0;

    /**
     * Makes an arm that can raise and lower.
     *
     * @param hardwareMap Hardware Map
     * @param telemetry   Telemetry
     */
    public ArmWithProfile(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        shoulderMotor = new Motor(hardwareMap, telemetry, "arm");
        configMotor(shoulderMotor);
        pidfController = new OLD_PIDFController(pidCoefficients, KV, KA, KS, new ArmFeedForwardWithProfile(this, KG));
    }

    /**
     * Configures the shoulder motor.
     *
     * @param motor the shoulder motor for the arm
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
    }

    /**
     * Gets the arm current in degrees to which the arm has moved.
     *
     * @return the current position in degrees to which the arm has moved
     */
    public double getCurrentAngle() {
        return shoulderMotor.getCurrentDegrees() + START_ANGLE;
    }

    /**
     * Gets the arm position in degrees to which the arm is moving.
     *
     * @return the target position in degrees to which the arm is moving
     */
    public double getTargetAngle() {
        return shoulderMotor.getTargetDegrees();
    }

    /**
     * Sets the shoulder motor to a position in degrees.
     */
    public void setTargetAngle(double angle) {
        double targetAngle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
        if (this.targetAngle != targetAngle) {
            this.targetAngle = targetAngle;
            profile = new MotionProfile(maxAcceleration,maxVelocity,getCurrentAngle(),targetAngle);
            pidfController.reset();
            atTargetCount = 0;
        }
    }

    /**
     * Sends power to the shoulder motor.
     */
    public void execute() {
        if(profile == null) {
            profile = new MotionProfile(maxAcceleration,maxVelocity,getCurrentAngle(),targetAngle);
            pidfController.reset();
        }
        double currentAngle = getCurrentAngle();
        double profileTargetPosition = profile.calculatePosition();
        pidfController.setTargetPosition(profileTargetPosition);
        double power = pidfController.update(profile.getTimestamp(), currentAngle);
        shoulderMotor.setPower(power);

        telemetry.addData("[Arm] ProfileTargetPos", profileTargetPosition);
        telemetry.addData("[Arm] Power", power);

        // Make sure the slide is at it's target for a number of consecutive loops. This is designed
        // to handle cases of "bounce" in the slide when moving to the target angle.
        double error = Math.abs(targetAngle - currentAngle);
        if (error <= TOLERABLE_ERROR) {
            atTargetCount++;
        } else {
            atTargetCount = 0;
        }
    }

    /**
     * Can tell if the current position is within a reasonable distance of the target.
     *
     * @return <code>true</code> if the arm is at the target angle; <code>false</code> otherwise.
     */
    public boolean isAtTarget() {
        boolean atTarget = atTargetCount >= AT_TARGET_COUNT;
        telemetry.addData("[Arm] atTarget", atTarget);
        return atTarget;
    }

    /**
     * Returns a string representation of the arm.
     *
     * @return a string representation of the arm
     */
    @Override
    @NonNull
    public String toString() {
        return "Arm{" +
                "target=" + targetAngle +
                ", current=" + getCurrentAngle() +
                "}";
    }

    /**
     * Gets an action that sets the angle of the arm.
     *
     * @param angle the target angle to which to set the arm
     * @return an action that sets the angle of the arm
     */
    public ActionEx actionSetTargetAngle(double angle) {
        return new SetTargetAngle(this, angle);
    }

    /**
     * An action that sets the angle of the arm.
     */
    public static class SetTargetAngle extends ActionExBase {
        private final ArmWithProfile arm;
        private final double angle;
        private boolean initialized = false;

        /**
         * Instantiates a new action to set the angle of the arm.
         *
         * @param arm   the arm to set the angle
         * @param angle the target angle for the arm
         */
        public SetTargetAngle(ArmWithProfile arm, double angle) {
            this.arm = arm;
            this.angle = angle;
        }

        /**
         * Moves the arm to the desired target angle.
         *
         * @param telemetryPacket telemetry that may be used to output data to the user
         * @return <code>true</code> if the arm is moving to the desired angle;
         *         <code>false</code> if the arm is at the desired angle
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialize();
            }

            return !arm.isAtTarget();
        }

        /**
         * Sets the target angle for the arm to the desired angle.
         */
        private void initialize() {
            arm.setTargetAngle(angle);
            initialized = true;
        }
    }
}
