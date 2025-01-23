package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionEx;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionExBase;
import org.firstinspires.ftc.teamcode.ftc7083.action.SequentialAction;
import org.firstinspires.ftc.teamcode.ftc7083.action.WaitAction;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Servo;

/**
 * The Claw class implements a claw used by the robot to pick up and score
 * samples and specimens.  By default the claw is closed or set to its zero
 * position initially.  Users can request that the claw be opened or moved
 * from its zero position to some angle specified in degrees.  The claw is
 * reset to its zero position when a close is requested.
 */
@Config
public class Claw extends SubsystemBase {
    public static String CLAW_SERVO = "claw";

    // Time to wait for the claw to open or close, based on observed time
    public static long CLAW_SERVO_TIME = 250; // milliseconds

    // Make default open/close degrees settable by FTC dashboard
    public static double CLOSE_DEGREE_OFFSET = 226;
    public static double DEFAULT_CLOSE_DEGREES = 14;
    public static double DEFAULT_OPEN_DEGREES = 55;

    // Make max claw degrees settable by FTC dashboard
    public static double MAX_CLAW_DEGREES = 355.0;

    private final Telemetry telemetry;

    private final ElapsedTime clawServoTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double clawAngle = -1;

    // Implement the claw using a Servo class
    private final Servo clawServo;

    /**
     * Constructor
     *
     * @param hardwareMap Servo configuration information
     * @param telemetry   telemetry used for output of information to the user.
     */
    public Claw(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.clawServo = new Servo(hardwareMap, CLAW_SERVO);
        this.clawServo.setDirection(Servo.Direction.REVERSE);
        clawServo.setMaxDegrees(MAX_CLAW_DEGREES);
        clawServo.close();
    }

    /**
     * Sets the claw to its opened position.
     */
    public void open() {
        setDegrees(DEFAULT_OPEN_DEGREES);
    }

    /**
     * Sets the claw to the specified number of degrees.
     */
    public void setDegrees(double degrees) {
        double adjustedDegrees = degrees + CLOSE_DEGREE_OFFSET;
        if (clawAngle != adjustedDegrees) {
            clawServo.setDegrees(adjustedDegrees);
            clawAngle = adjustedDegrees;
            clawServoTimer.reset();
        }
    }

    /**
     * Get the current position of the claw.
     *
     * @return The current position of the claw in degrees.
     */
    public double getCurrentPosition() {
        return clawServo.getPosition();
    }

    /**
     * Get the current degrees of the claw.
     *
     * @return the current degrees for the claw.
     */
    public double getCurrentDegrees() {
        return clawServo.getDegrees() - CLOSE_DEGREE_OFFSET;
    }

    /**
     * Sets the claw to its closed position.
     */
    public void close() {
        setDegrees(DEFAULT_CLOSE_DEGREES);
    }

    /**
     * Checks if the claw is at the target position.
     */
    public boolean isAtTarget() {
        double elapsedTime = clawServoTimer.time();
        boolean atTarget = elapsedTime >= CLAW_SERVO_TIME;
        telemetry.addData("[Claw] atTarget", atTarget);
        return atTarget;
    }

    @NonNull
    public String toString() {
        return "Claw{" +
                "angle=" + clawAngle +
                ", atTarget=" + isAtTarget() +
                "}";
    }

    /**
     * Gets an action to open the claw. This action does not wait for the claw to be successfully
     * opened.
     *
     * @return an action to open the claw
     */
    public ActionEx actionOpenClaw() {
        return new OpenClaw(this);
    }

    /**
     * Gets an action to open the claw. This action waits for the claw to be successfully opened.
     *
     * @return an action to open the claw
     */
    public ActionEx actionOpenClawWithWait() {
        return new SequentialAction(
                new OpenClaw(this),
                new WaitAction(CLAW_SERVO_TIME)
        );
    }

    /**
     * Gets an action to close the claw. This action does not wait for the claw to be successfully
     * closed.
     *
     * @return an action to close the claw
     */
    public ActionEx actionCloseClaw() {
        return new CloseClaw(this);
    }

    /**
     * Gets an action to close the claw. This action waits for the claw to be successfully closed.
     *
     * @return an action to close the claw
     */
    public ActionEx actionCloseClawWithWait() {
        return new SequentialAction(
                new CloseClaw(this),
                new WaitAction(CLAW_SERVO_TIME)
        );
    }

    /**
     * An action that opens the claw.
     */
    public static class OpenClaw extends ActionExBase {
        private final Claw claw;
        private boolean initialized = false;

        /**
         * Instantiates a new action to open the claw.
         *
         * @param claw the claw to be opened
         */
        public OpenClaw(Claw claw) {
            this.claw = claw;
        }

        /**
         * Initializes the claw, setting the claw to the <code>opened</code> position.
         */
        private void initialize() {
            claw.open();
            initialized = true;
        }

        /**
         * Opens the claw and completes execution.
         *
         * @param telemetryPacket telemetry used for output of data to the user
         * @return <code>false</code>, which causes the action to complete
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialize();
            }
            return false;
        }
    }

    /**
     * An action that closes the claw.
     */
    public static class CloseClaw extends ActionExBase {
        private final Claw claw;
        private boolean initialized = false;

        public CloseClaw(Claw claw) {
            this.claw = claw;
        }

        /**
         * Initializes the claw, setting the claw to the <code>closed</code> position.
         */
        private void initialize() {
            claw.close();
            initialized = true;
        }

        /**
         * Closes the claw and completes execution.
         *
         * @param telemetryPacket telemetry used for output of data to the user
         * @return <code>false</code>, which causes the action to complete
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialize();
            }
            return false;
        }
    }
}
