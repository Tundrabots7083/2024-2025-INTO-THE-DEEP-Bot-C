package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionEx;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionExBase;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.GainSchedulingPIDController;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.LookUpTableArgs;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;

/**
 * An Arm is used to move the scoring subsystem in a circular arc, allowing the robot to both
 * pickup and score sample and specimens.
 */
@Config
public class Arm extends SubsystemBase {
    public static double START_ANGLE = -47.0;
    public static double ACHIEVABLE_MAX_RPM_FRACTION = 1.0;
    public static double TICKS_PER_REV = 1397.1; // GoBuilda ticks per rev for 60 rpm
    public static double TOLERABLE_ERROR = 0.7; // In degrees
    public static double MIN_ANGLE = -47.0;
    public static double MAX_ANGLE = 90.0;

    private final Motor shoulderMotor;
    private final Telemetry telemetry;
    private double feedforward;
    private final GainSchedulingPIDController gainSchedulingPIDController;
    public double GEARING = 22.0 / 10.0;
    private double targetAngle = START_ANGLE;

    /**
     * Makes an arm that can raise and lower.
     *
     * @param hardwareMap Hardware Map
     * @param telemetry   Telemetry
     */
    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, 0);
    }

    /**
     * Makes an arm that can raise and lower.
     *
     * @param hardwareMap Hardware Map
     * @param telemetry   Telemetry
     * @param feedforward feedforward
     */
    public Arm(HardwareMap hardwareMap, Telemetry telemetry, double feedforward) {
        this.telemetry = telemetry;
        this.feedforward = feedforward;
        shoulderMotor = new Motor(hardwareMap, telemetry, "arm");
        configMotor(shoulderMotor);

        LookUpTableArgs[] kpLUTArgs = new LookUpTableArgs[]{
                new LookUpTableArgs(-59, 0.12),
                new LookUpTableArgs(210,0.12)};
        LookUpTableArgs[] kiLUTArgs = new LookUpTableArgs[]{
                new LookUpTableArgs(-59,0.0),
                new LookUpTableArgs(210,0.0)};
        LookUpTableArgs[] kdLUTArgs = new LookUpTableArgs[]{
                new LookUpTableArgs(-59, 0),
                new LookUpTableArgs(-20, 0),
                new LookUpTableArgs(0, 0),
                new LookUpTableArgs(20, 0),
                new LookUpTableArgs(90, 0),
                new LookUpTableArgs(120, 0),
                new LookUpTableArgs(200,0)};

        gainSchedulingPIDController = new GainSchedulingPIDController(kpLUTArgs, kiLUTArgs, kdLUTArgs);
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
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
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
            gainSchedulingPIDController.reset();
        }
    }

    public void setFeedforward (double feedforward) {this.feedforward = feedforward;}

    /**
     * Sends power to the shoulder motor.
     */
    public void execute() {
        double degrees = shoulderMotor.getCurrentDegrees() + START_ANGLE;
        double power = gainSchedulingPIDController.calculate(targetAngle, degrees) + this.feedforward;

        shoulderMotor.setPower(power);
        telemetry.addData("[Arm] Target", targetAngle);
        telemetry.addData("[Arm] Current", degrees);
    }

    /**
     * Can tell if the current position is within a reasonable distance of the target.
     *
     * @return <code>true</code> if the arm is at the target angle; <code>false</code> otherwise.
     */
    public boolean isAtTarget() {
        double degrees = shoulderMotor.getCurrentDegrees() + START_ANGLE;
        double error = Math.abs(targetAngle - degrees);
        return error <= TOLERABLE_ERROR;
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
        private final Arm arm;
        private final double angle;
        private boolean initialized = false;

        /**
         * Instantiates a new action to set the angle of the arm.
         *
         * @param arm   the arm to set the angle
         * @param angle the target angle for the arm
         */
        public SetTargetAngle(Arm arm, double angle) {
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

            arm.execute();
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
