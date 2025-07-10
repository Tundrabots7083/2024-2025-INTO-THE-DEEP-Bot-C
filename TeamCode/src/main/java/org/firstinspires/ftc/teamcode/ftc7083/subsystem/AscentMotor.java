package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.FeedForward;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.profile.MotionProfile;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;

/**
 * A linear slide can extend and retract the wrist and claw attached to the robot's scoring
 * subsystem.
 */
@Config
public class AscentMotor extends SubsystemBase {

    // Constants for determining if the slide is at target
    public static double TOLERABLE_ERROR = 1.0; // inches

    private final Motor ascentMotor;
    private LinearSlide realLinearSlide;
    private final Telemetry telemetry;
    private double linearSlideTargetLength = 5;
    private double currentLinearSlideLength;
    public boolean isAscending = false;
    private boolean atTarget = false;


    /**
     * Instantiates the linear slide for the robot with the specified feed forward.
     *
     * @param hardwareMap  Hardware Map
     * @param telemetry    Telemetry
     */
    public AscentMotor(LinearSlide linearSlide, HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        realLinearSlide = linearSlide;
        currentLinearSlideLength = realLinearSlide.getCurrentLength();
        ascentMotor = new Motor(hardwareMap, telemetry, "ascentMotor");
        configMotor(ascentMotor);
    }


    /**
     * Gets the slide length in inches
     * Finds the value for the length
     *
     * @return slide length in inches
     */
    public double getCurrentLinearSlideLength() {
        return realLinearSlide.getCurrentLength();
    }

    /**
     * sets the ascent motor's "state" to ascending
     */
    public void ascend() {isAscending = true;}

    /**
     * Configures the motor used for the linear slide
     *
     * @param motor the motor to be configured
     */
    private void configMotor(Motor motor) {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motor.setMotorType(motorConfigurationType);
        motor.setMode(Motor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(Motor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(Motor.Direction.FORWARD);
    }

    /**
     * sets the power for the pid controller
     */
    @Override
    public void execute() {

        if(isAscending) {
            currentLinearSlideLength = getCurrentLinearSlideLength();
            double error = currentLinearSlideLength - linearSlideTargetLength;
            double power = 0;
            if (error <= TOLERABLE_ERROR) {
                power = 1;
            } else {
                atTarget = true;
            }
            ascentMotor.setPower(power);

            telemetry.addData("[Slide] PID Target", this.currentLinearSlideLength);
            telemetry.addData("[Slide] Power", power);
        }
    }

    /**
     * returns whether the motor is at target
     */
    public boolean isAtTarget() {
        telemetry.addData("[Slide] current", currentLinearSlideLength);
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
        return "AscentMotor{" +", current=" + getCurrentLinearSlideLength() +
                "}";
    }
}
