package org.firstinspires.ftc.teamcode.ftc7083.subsystem;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Servo;

/**
 * Wrist implements the servo operated wrist on the arm.
 */
@Config
public class Wrist extends SubsystemBase {
    public static int MIN_ROLL = -90;
    public static int MAX_ROLL = 90;
    public static int MIN_PITCH = -90;
    public static int MAX_PITCH = 90;

    private final Telemetry telemetry;

    private double pitch = 0.0;
    private double roll = 0.0;
    private final Servo pitchServo;
    private final Servo rollServo;

    /**
     * Wrist initializes a new wrist as well as initializing all servos to be used.
     *
     * @param hardwareMap the hardware map that contains the servo hardware.
     * @param telemetry   the telemetry used to display data on the driver station.
     */
    public Wrist(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        this.telemetry = telemetry;
        pitchServo = new Servo(hardwareMap, "wristRoll");
        pitchServo.setMaxDegrees(355);
        rollServo = new Servo(hardwareMap, "wristPitch");
        rollServo.setMaxDegrees(355);
    }

    /**
     * setPitch sets the target for the pitch servo.
     *
     * @param pitch the final pitch target in degrees.
     */
    public void setPitch(double pitch) {
        this.pitch = pitch;
    }

    /**
     * setYaw sets the target for the yaw servo.
     *
     * @param roll the final yaw target in degrees.
     */
    public void setRoll(double roll) {
        this.roll = roll;
    }

    /**
     * This method calculates the values to assign to the frontServo and backServo
     * given the pitch and yaw and sets the servos to their respective values.
     *
     * @param pitch wrist pitch
     * @param yaw   wrist roll
     */
    /*
    public void setPosition(double pitch, double yaw) {
        this.roll = Range.clip(yaw, MIN_ROLL, MAX_ROLL);
        this.pitch = Range.clip(pitch, MIN_PITCH, MAX_PITCH);

        double frontServoYaw = 90 - this.roll;
        double backServoYaw = 90 - this.roll;

//        double frontServoPosition = (frontServoYaw - this.pitch) <= 180 ? (frontServoYaw - this.pitch) : 180;
//        double backServoPosition = (backServoYaw + this.pitch) <= 180 ? (backServoYaw + this.pitch) : 180;
        double frontServoPosition = frontServoYaw - this.pitch;
        double backServoPosition = backServoYaw + this.pitch;

        pitchServo.setDegrees(frontServoPosition);
        rollServo.setDegrees(backServoPosition);

        telemetry.addData("[Wrist] yaw", this.roll);
        telemetry.addData("[Wrist] pitch", this.pitch);
        telemetry.addData("[Wrist] front servo", frontServoPosition);
        telemetry.addData("[Wrist] back servo", backServoPosition);
    }
    */
    /**
     * getPitchPosition returns the current set pitch position.
     *
     * @return returns pitch position
     */
    public double getPitch() {
        return this.pitch;
    }

    /**
     * getYawPosition returns the current set yaw position.
     *
     * @return returns yaw position
     */
    public double getRoll() {
        return this.roll;
    }

    /**
     * Returns the position of the front servo in degrees.
     *
     * @return frontServo degrees
     */
    public double getPitchServoDegrees() {
        return pitchServo.getDegrees();
    }

    /**
     * Returns the position of the back servo in degrees.
     *
     * @return backServo degrees
     */
    public double getRollServoDegrees() {
        return rollServo.getDegrees();
    }
}
