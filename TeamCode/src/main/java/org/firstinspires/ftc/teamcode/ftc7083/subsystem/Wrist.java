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
    // Maximum number of degrees supported by the pitch and roll servos
    public static double PITCH_SERVO_MAX_DEGREES = 355;
    public static double ROLL_SERVO_MAX_DEGREES = 355;

    // Offset for the pitch and roll servos to make the "zero" position the middle
    public static double PITCH_DEGREES_OFFSET = 150.0;
    public static double ROLL_DEGREES_OFFSET = 156.0;

    // Minimum and maximum pitch and roll values we allow to be set
    public static double MIN_ROLL = -90;
    public static double MAX_ROLL = 90;
    public static double MIN_PITCH = 0;
    public static double MAX_PITCH = 180;

    // Pre-set positions
    public static double INTAKE_SAMPLE_PITCH = 0.0;
    public static double INTAKE_SAMPLE_ROLL = 0.0;
    public static double INTAKE_SPECIMEN_PITCH = 0.0;
    public static double INTAKE_SPECIMEN_ROLL = 100.0;
    public static double SCORE_BASKET_PITCH = 120.0;
    public static double SCORE_BASKET_ROLL = 0.0;
    public static double SCORE_CHAMBER_PITCH = 100.0;
    public static double SCORE_CHAMBER_ROLL = 0.0;
    public static double START_POSITION_PITCH = 0.0;
    public static double START_POSITION_ROLL = 0.0;

    private final Telemetry telemetry;

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
        pitchServo = new Servo(hardwareMap, "wristPitch");
        pitchServo.setMaxDegrees(PITCH_SERVO_MAX_DEGREES);
        pitchServo.setDirection(Servo.Direction.REVERSE);
        rollServo = new Servo(hardwareMap, "wristRoll");
        rollServo.setMaxDegrees(ROLL_SERVO_MAX_DEGREES);
        rollServo.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * Sets the wrist to its starting position.
     */
    public void setToStartPosition() {
        setPitchDegrees(START_POSITION_PITCH);
        setRollDegrees(START_POSITION_ROLL);
    }

    /**
     * Sets the wrist so that it can intake a sample off the ground.
     */
    public void setToIntakeSample() {
        setPitchDegrees(INTAKE_SAMPLE_PITCH);
        setRollDegrees(INTAKE_SAMPLE_ROLL);
    }

    /**
     * Sets the wrist so that it can intake a specimen off the wall.
     */
    public void setToIntakeSpecimen() {
        setPitchDegrees(INTAKE_SPECIMEN_PITCH);
        setRollDegrees(INTAKE_SPECIMEN_ROLL);
    }

    /**
     * Sets the wrist for scoring in the high basket.
     */
    public void setToScoreBasket() {
        setPitchDegrees(SCORE_BASKET_PITCH);
        setRollDegrees(SCORE_BASKET_ROLL);
    }

    /**
     * Sets the wrist for scoring on the high chamber.
     */
    public void setToScoreChamber() {
        setPitchDegrees(SCORE_CHAMBER_PITCH);
        setRollDegrees(SCORE_CHAMBER_ROLL);
    }

    /**
     * getPitchPosition returns the current set pitch position.
     *
     * @return returns pitch position
     */
    public double getPitchPosition() {
        return pitchServo.getPosition();
    }

    /**
     * getYawPosition returns the current set yaw position.
     *
     * @return returns yaw position
     */
    public double getRollPosition() {
        return rollServo.getPosition();
    }

    /**
     * Set the number of degrees for the pitch servo.
     *
     * @param degrees the number of degrees for the pitch servo
     */
    public void setPitchDegrees(double degrees) {
        double pitch = Range.clip(degrees, MIN_PITCH, MAX_PITCH) + PITCH_DEGREES_OFFSET;
        pitchServo.setDegrees(pitch);
    }

    /**
     * Returns the position of the front servo in degrees.
     *
     * @return frontServo degrees
     */
    public double getPitchDegrees() {
        return pitchServo.getDegrees() - PITCH_DEGREES_OFFSET;
    }

    /**
     * Set the number of degrees for the roll servo.
     *
     * @param degrees the number of degrees for the roll servo
     */
    public void setRollDegrees(double degrees) {
        double roll = Range.clip(degrees, MIN_ROLL, MAX_ROLL) + ROLL_DEGREES_OFFSET;
        rollServo.setDegrees(roll);
    }

    /**
     * Returns the position of the back servo in degrees.
     *
     * @return backServo degrees
     */
    public double getRollDegrees() {
        return rollServo.getDegrees() - ROLL_DEGREES_OFFSET;
    }

    @NonNull
    public String toString() {
        return "Wrist{" +
                "pitch=" + pitchServo.getDegrees() +
                ", roll=" + rollServo.getDegrees() +
                "}";
    }
}
