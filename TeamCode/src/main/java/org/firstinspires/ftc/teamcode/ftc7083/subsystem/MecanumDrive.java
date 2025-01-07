package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;

import java.util.Arrays;
import java.util.Collection;

/**
 * MecanumDrive implements the drive chassis for the robot.
 */
@Config
public class MecanumDrive extends SubsystemBase {
    public static double POWER_EXPONENT = 1.0;
    public static double STRAFING_ADJUSTMENT = 1.1;
    public static double MAX_DRIVE_POWER_GAIN = 1.0;

    private final Telemetry telemetry;
    private final Motor rightFront, rightRear, leftFront, leftRear;
    private double driveGain = MAX_DRIVE_POWER_GAIN;

    /**
     * MecanumDrive initializes a new mecanum drive train.
     *
     * @param hardwareMap the hardware map that contains the drone launcher hardware.
     * @param telemetry   the telemetry used to display data on the driver station.
     */
    public MecanumDrive(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        this.telemetry = telemetry;

        leftFront = new Motor(hardwareMap, telemetry, "leftFrontWheel");
        leftRear = new Motor(hardwareMap, telemetry, "leftRearWheel");
        rightFront = new Motor(hardwareMap, telemetry, "rightFrontWheel");
        rightRear = new Motor(hardwareMap, telemetry, "rightRearWheel");

        leftFront.setDirection(Motor.Direction.REVERSE);
        leftRear.setDirection(Motor.Direction.REVERSE);

        Collection<Motor> motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);
        for (Motor motor : motors) {
            initMotor(motor);
        }
    }

    /**
     * initMotor initializes a motor attached to the mecanum wheel.
     *
     * @param motor the motor to be initialized.
     */
    private void initMotor(@NonNull Motor motor) {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(motorConfigurationType);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setMode(Motor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * drive sets the powers to the wheel motors to result in the robot moving
     * in the direction provided on input.
     *
     * @param x    how much to move right or, if a negative value, left.
     * @param y    how much to move forward or, if a negative value, backward.
     * @param turn how much to rotate the robot.
     */
    public void drive(double x, double y, double turn) {
        x *= STRAFING_ADJUSTMENT; // Adjust for imperfect strafing

        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontPower = power * cos / max - turn;
        double rightFrontPower = power * sin / max + turn;
        double leftRearPower = power * sin / max - turn;
        double rightRearPower = power * cos / max + turn;

        // Normalize motor powers to ensure none exceeds 1.0
        double maxMotorPower = power + Math.abs(turn);
        if (maxMotorPower > 1) {
            leftFrontPower /= maxMotorPower;
            rightFrontPower /= maxMotorPower;
            leftRearPower /= maxMotorPower;
            rightRearPower /= maxMotorPower;
        }

        // Adjust the power applied to the motors. This raises the power to an exponent to create
        // an exponential curve, as well as applies the power gain to each motor's power.
        leftFrontPower = adjustMotorPower(leftFrontPower);
        rightFrontPower = adjustMotorPower(rightFrontPower);
        leftRearPower = adjustMotorPower(leftRearPower);
        rightRearPower = adjustMotorPower(rightRearPower);

        setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);
    }

    /**
     * Adjust the value to reduce sensitivity of the joystick when pushed only a small distance
     *
     * @param value the value based from the controller
     * @return the adjusted value
     */
    private double adjustMotorPower(double value) {
        double sign = value < 0 ? -1 : 1;
        double newPower = Math.pow(Math.abs(value), POWER_EXPONENT) * sign;
        newPower *= driveGain;
        return newPower;
    }

    /**
     * setMotorPowers sets the power for the wheels, normalizing for a maximum power of 1.0.
     *
     * @param leftFrontPower  the power for the left front motor.
     * @param leftRearPower   the power for the left rear motor
     * @param rightRearPower  the power for the right rear motor.
     * @param rightFrontPower the power for the right front motor.
     */
    public void setMotorPowers(double leftFrontPower, double leftRearPower, double rightRearPower, double rightFrontPower) {
        // Get the maximum power for any motor, or 1.0, whichever is greater
        double maxPower = maxAbs(1.0, leftFrontPower, leftRearPower, rightFrontPower, rightRearPower);

        // Divide by the maximum power, which guarantees no motor's power will exceed 1.0.
        // This also ensures that all motors get a proportional amount of power should the
        // input power for any motor exceed 1.0.
        leftFrontPower /= maxPower;
        leftRearPower /= maxPower;
        rightFrontPower /= maxPower;
        rightRearPower /= maxPower;

        // Now that the power have been normalized, go ahead and set power for the motors.
        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);

        telemetry.addData("[DRIVE] Left Front Power", leftFrontPower);
        telemetry.addData("[DRIVE] Left Rear Power", leftRearPower);
        telemetry.addData("[DRIVE] Right Front Power", rightFrontPower);
        telemetry.addData("[DRIVE] Right Rear Power", rightRearPower);
    }

    /**
     * Turns the robot given the amount of power.
     *
     * @param turnPower the amount of power to apply to the wheel motors.
     */
    public void turn(double turnPower) {

        double leftFrontPower = turnPower;
        double rightFrontPower = turnPower;
        double leftRearPower = turnPower;
        double rightRearPower = turnPower;

        // Normalize motor powers to ensure none exceeds 1.0
        double maxMotorPower = Math.abs(turnPower);
        if (maxMotorPower > 1) {
            leftFrontPower /= maxMotorPower;
            rightFrontPower /= maxMotorPower;
            leftRearPower /= maxMotorPower;
            rightRearPower /= maxMotorPower;
        }

        setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);
    }

    /**
     * Drives the robot in a straight direction.
     *
     * @param drivePower the amount of power to apply to the wheel motors.
     */
    public void driveStraight(double drivePower) {

        double leftFrontPower = drivePower;
        double rightFrontPower = drivePower;
        double leftRearPower = drivePower;
        double rightRearPower = drivePower;

        // Normalize motor powers to ensure none exceeds 1.0
        double maxMotorPower = Math.abs(drivePower);
        if (maxMotorPower > 1) {
            leftFrontPower /= maxMotorPower;
            rightFrontPower /= maxMotorPower;
            leftRearPower /= maxMotorPower;
            rightRearPower /= maxMotorPower;
        }

        setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);

    }

    /**
     * Gets the current power gain for the drive subsystem.
     *
     * @return the current power gain for the drive subsystem
     */
    public double getDriveGain() {
        return driveGain;
    }

    /**
     * Set the power gain for the drive subsystem. The power applied to the wheels is multiplied
     * by the gain to adjust the power that is set.
     *
     * @param gain the power gain to use
     * @return this mecanum drive instance
     */
    public MecanumDrive setDriveGain(double gain) {
        // Gain is required to be between [0,1]
        if (gain > MAX_DRIVE_POWER_GAIN) {
            gain = MAX_DRIVE_POWER_GAIN;
        } else if (gain < 0) {
            gain = 0;
        }
        this.driveGain = gain;
        return this;
    }

    /**
     * Gets a string representation of this mecanum drive.
     *
     * @return a string representation of this mecanum drive
     */
    @NonNull
    @Override
    public String toString() {
        return "MecanumDrive{" +
                "rightFront=" + rightFront +
                ", rightRear=" + rightRear +
                ", leftFront=" + leftFront +
                ", leftRear=" + leftRear +
                '}';
    }
}
