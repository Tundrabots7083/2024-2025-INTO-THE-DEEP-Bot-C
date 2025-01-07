package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.PIDController;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.PIDControllerEx;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;

/**
 * A linear slide can extend and retract the wrist and claw attached to the robot's scoring
 * subsystem.
 */
@Config
public class LinearSlide extends SubsystemBase {
    public static double SPOOL_DIAMETER = 1.4; // in inches
    public static double TICKS_PER_REV = 537.7;
    public static double ACHIEVABLE_MAX_RPM_FRACTION = 1.0;
    public static double KP = 0.3;
    public static double KI = 0.4;
    public static double KD = 0.05;
    public static double KG = 0.3;
    public static double TOLERABLE_ERROR = 0.05; // inches
    public static double MIN_EXTENSION_LENGTH = 0.3;
    public static double MAX_EXTENSION_LENGTH = 18;
    private final Motor slideMotor;
    private final Telemetry telemetry;
    private final PIDController pidController;
    public double GEARING = 1.0;
    private double targetLength = 0;

    /**
     * Instantiates the linear slide for the robot.
     *
     * @param hardwareMap Hardware Map
     * @param telemetry   Telemetry
     */
    public LinearSlide(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        slideMotor = new Motor(hardwareMap, telemetry, "linearSlide");
        configMotor(slideMotor);
        pidController = new PIDControllerEx(KP, KI, KD, KG);
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
        if (length != targetLength) {
            telemetry.addData("[Slide] clipped", length);
        }
        if (this.targetLength != targetLength) {
            this.targetLength = targetLength;
            pidController.reset();
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
        if (!isAtTarget()) {
            double power = pidController.calculate(targetLength, getCurrentLength());
            slideMotor.setPower(power);
            telemetry.addData("[Slide] power", power);
            telemetry.addData("[Slide] inches", getCurrentLength());
            telemetry.addData("[Slide] ticks", slideMotor.getCurrentPosition());
        }
    }

    /**
     * checks if the length is within the tolerable error and if it is then the motor will stop
     */
    public boolean isAtTarget() {
        double error = Math.abs(targetLength - getCurrentLength());
        telemetry.addData("[Slide] error", error);
        telemetry.addData("[Slide] target", targetLength);
        telemetry.addData("[Slide] current", getCurrentLength());
        return error <= TOLERABLE_ERROR;
    }
}
