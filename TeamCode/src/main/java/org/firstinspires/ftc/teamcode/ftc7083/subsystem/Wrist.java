package org.firstinspires.ftc.teamcode.ftc7083.subsystem;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Servo;

/**
 * Wrist implements the servo operated wrist on the arm.
 */
@Config
public class Wrist extends SubsystemBase {
    public static int MIN_YAW = -90;
    public static int MAX_YAW = 90;
    public static int MIN_PITCH = -90;
    public static int MAX_PITCH = 90;

    private final Telemetry telemetry;

    private double pitch = 0.0;
    private double yaw = 0.0;
    private final Servo frontServo;
    private final Servo backServo;

    /**
     * Wrist initializes a new wrist as well as initializing all servos to be used.
     *
     * @param hardwareMap the hardware map that contains the servo hardware.
     * @param telemetry   the telemetry used to display data on the driver station.
     */
    public Wrist(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        this.telemetry = telemetry;
        frontServo = new Servo(hardwareMap, "wristRoll");
        frontServo.setMaxDegrees(180);
        backServo = new Servo(hardwareMap, "wristPitch");
        backServo.setMaxDegrees(180);
    }

    /**
     * setPitch sets the target for the pitch servo.
     *
     * @param pitch the final pitch target in degrees.
     */
    public void setPitch(double pitch) {
        setPosition(pitch, this.yaw);
    }

    /**
     * setYaw sets the target for the yaw servo.
     *
     * @param yaw the final yaw target in degrees.
     */
    public void setYaw(double yaw) {
        setPosition(this.pitch, yaw);
    }

    /**
     * This method calculates the values to assign to the frontServo and backServo
     * given the pitch and yaw and sets the servos to their respective values.
     *
     * @param pitch wrist pitch
     * @param yaw   wrist yaw
     */
    public void setPosition(double pitch, double yaw) {
        this.yaw = Range.clip(yaw, MIN_YAW, MAX_YAW);
        this.pitch = Range.clip(pitch, MIN_PITCH, MAX_PITCH);

        double frontServoYaw = 90 - this.yaw;
        double backServoYaw = 90 - this.yaw;

//        double frontServoPosition = (frontServoYaw - this.pitch) <= 180 ? (frontServoYaw - this.pitch) : 180;
//        double backServoPosition = (backServoYaw + this.pitch) <= 180 ? (backServoYaw + this.pitch) : 180;
        double frontServoPosition = frontServoYaw - this.pitch;
        double backServoPosition = backServoYaw + this.pitch;

        frontServo.setDegrees(frontServoPosition);
        backServo.setDegrees(backServoPosition);

        telemetry.addData("[Wrist] yaw", this.yaw);
        telemetry.addData("[Wrist] pitch", this.pitch);
        telemetry.addData("[Wrist] front servo", frontServoPosition);
        telemetry.addData("[Wrist] back servo", backServoPosition);
    }

    /**
     * getPitchPosition returns the current set pitch position.
     *
     * @return returns pitch position
     */
    public double getPitchPosition() {
        return this.pitch;
    }

    /**
     * getYawPosition returns the current set yaw position.
     *
     * @return returns yaw position
     */
    public double getYawPosition() {
        return this.yaw;
    }

    /**
     * Returns the position of the front servo in degrees.
     *
     * @return frontServo degrees
     */
    public double getFrontServoDegrees() {
        return frontServo.getDegrees();
    }

    /**
     * Returns the position of the back servo in degrees.
     *
     * @return backServo degrees
     */
    public double getBackServoDegrees() {
        return backServo.getDegrees();
    }
}
