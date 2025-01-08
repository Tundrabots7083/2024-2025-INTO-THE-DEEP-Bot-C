package org.firstinspires.ftc.teamcode.ftc7083.subsystem.controller;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.MecanumDrive;

/**
 * A controller for a mecanum drive chassis. The following controls on <em>gamepad1</em> are used
 * for by the drive subsystem.
 * <ul>
 *     <li>
 *         <em>gamepad1.left_stick</em>: used to move the robot forward and backwards, and strafe left
 *         and right. The Y value is used for moving forward and backwards; the X value is used for
 *         moving left and right.
 *     </li>
 *     <li>
 *         <em>gamepad1.right_stick</em>: rotate the robot clockwise or counterclockwise. The X value
 *         is used to determine the rotation rate; a positive value rotates the robot clockwise, while
 *         a negative value rotates the robot counterclockwise.
 *     </li>
 *     <li>
 *         <em>gamepad1.left_bumper</em>: toggle the drive speed gain between a <em>fast</em> and
 *         <em>slow</em> mode.
 *     </li>
 * </ul>
 */
@Config
public class MecanumDriveController implements SubsystemController {
    public static double TURN_MULTIPLIER = 0.75; // Max turning speed multiplier
    public static double DRIVE_GAIN_MAX = 1; // Fast drive speed gain
    public static double DRIVE_GAIN_MIN = 0.4; // Slow drive speed gain

    public static boolean USE_TIERED_GAMEPAD_VALUES = false;
    public static double TIER_9_PERCENT = 0.0;
    public static double TIER_18_PERCENT = 0.1;
    public static double TIER_27_PERCENT = 0.2;
    public static double TIER_36_PERCENT = 0.3;
    public static double TIER_45_PERCENT = 0.4;
    public static double TIER_54_PERCENT = 0.5;
    public static double TIER_63_PERCENT = 0.6;
    public static double TIER_72_PERCENT = 0.7;
    public static double TIER_81_PERCENT = 0.8;
    public static double TIER_90_PERCENT = 0.9;
    public static double TIER_100_PERCENT = 1.0;

    private final MecanumDrive mecanumDrive;
    private final Telemetry telemetry;

    private boolean previousLeftBumper = false;
    private boolean fastMode;

    /**
     * Creates a controller for a mecanum drive chassis.
     *
     * @param mecanumDrive the mecanum drive to control.
     * @param telemetry    the telemetry used to display data on the driver station.
     */
    public MecanumDriveController(MecanumDrive mecanumDrive, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.mecanumDrive = mecanumDrive;

        mecanumDrive.setDriveGain(DRIVE_GAIN_MAX);
        fastMode = true;
    }

    /**
     * Updates the percentage of the power that is applied to the drive train. Ths affects
     * the X and Y axis (driveGain) and the turn speed (turnGain).
     *
     * @param gamepad the gamepad with the controls for adjusting the robot gain
     */
    private void setGain(Gamepad gamepad) {
        // Toggle the drive gain as the left bumper is pressed
        if (gamepad.left_bumper && !previousLeftBumper) {
            if (fastMode) {
                mecanumDrive.setDriveGain(DRIVE_GAIN_MIN);
                fastMode = false;
            } else {
                mecanumDrive.setDriveGain(DRIVE_GAIN_MAX);
                fastMode = true;
            }
        }
        previousLeftBumper = gamepad.left_bumper;
    }

    /**
     * Updates the robot drive speed based on the left and right joysticks on gamepad1. In addition,
     * the left bumper on gamepad1 can be used to toggle the gain for the X axis, Y axis and
     * turn speed.
     *
     * @param gamepad1 Gamepad1
     * @param gamepad2 Gamepad2
     */
    public void execute(Gamepad gamepad1, Gamepad gamepad2) {
        setGain(gamepad1);

        double x = modifyGamepadValue(gamepad1.left_stick_x);
        double y = modifyGamepadValue(-gamepad1.left_stick_y);
        double turn = modifyGamepadValue(-gamepad1.right_stick_x) * TURN_MULTIPLIER;
        telemetry.addData("[DRIVE] X", x);
        telemetry.addData("[DRIVE] Y", y);
        telemetry.addData("[DRIVE] Turn", turn);

        mecanumDrive.drive(x, y, turn);
    }

    /**
     * Convert the analog joystick values to tiered values.
     *
     * @param value the analog joystick value
     * @return the corresponding tiered value.
     */
    private double modifyGamepadValue(double value) {
        // Don't modify the input value at all
        if (!USE_TIERED_GAMEPAD_VALUES) {
            return value;
        }

        // Modify the analog value to a value based on the tier. Each "tier" is roughly 9% of the
        // overall analog input values, and any control value within that "tier" is treated
        // equally.
        double sign = value < 0.0 ? -1.0 : 1.0;
        double absValue = Math.abs(value);
        if (absValue <= 0.09) {
            return sign * TIER_9_PERCENT;
        } else if (absValue <= 0.18) {
            return  sign * TIER_18_PERCENT;
        } else if (absValue <= 0.27) {
            return sign * TIER_27_PERCENT;
        } else if (absValue <= 0.36) {
            return sign * TIER_36_PERCENT;
        } else if (absValue <= 0.45) {
            return sign * TIER_45_PERCENT;
        } else if (absValue <= 0.54) {
            return sign * TIER_54_PERCENT;
        } else if (absValue <= 0.63) {
            return sign * TIER_63_PERCENT;
        } else if (absValue < 0.72) {
            return sign * TIER_72_PERCENT;
        } else if (absValue <= 0.81) {
            return sign * TIER_81_PERCENT;
        } else if (absValue <= 0.90) {
            return sign * TIER_90_PERCENT;
        } else {
            return sign * TIER_100_PERCENT;
        }
    }

    /**
     * Gets a string representation of this mecanum drive controller.
     *
     * @return a string representation of this mecanum drive controller
     */
    @NonNull
    @Override
    public String toString() {
        return "MecanumDriveController{" +
                "mecanumDrive=" + mecanumDrive +
                "fastMode=" + fastMode +
                '}';
    }
}
