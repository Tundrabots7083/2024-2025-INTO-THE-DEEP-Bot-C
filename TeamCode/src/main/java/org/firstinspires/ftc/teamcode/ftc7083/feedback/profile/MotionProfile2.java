package org.firstinspires.ftc.teamcode.ftc7083.feedback.profile;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

/**
 * A trapezoidal motion profile. This involves defining acceleration, constant velocity, and
 * deceleration phases to ensure smooth motion.
 */
@Config
public class MotionProfile2 {
    // Constant for first pass of the motion profile
    public static double DEFAULT_TIMESTEP = 0.01; // 100 milliseconds in seconds

    // Constants that help determine when the motion profile isAtTarget
    public static double TOLERABLE_POSITION_ERROR = 0.01;
    public static double TOLERABLE_VELOCITY_ERROR = 0.01;

    // Values specific to the hardware component being controlled
    private final double maxVelocity;      // Maximum velocity in units per second
    private final double acceleration;     // Acceleration in units per second^2
    private final double targetPosition;   // Target position in units
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);  // Timer for each step, in seconds

    // Current values for the motion profile
    private double currentPosition;        // Current position in units
    private double currentVelocity;        // Current velocity in units per second
    private boolean hasRun = false;        // The motion profile has run at least once

    /**
     * Instantiates a motion trapezoidal motion profile.
     * @param acceleration    the acceleration in units per second^2
     * @param maxVelocity     the maximum velocity in units per second
     * @param currentPosition the current position in units
     * @param targetPosition  the target position in units
     */
    public MotionProfile2(double acceleration, double maxVelocity, double currentPosition, double targetPosition) {
        this.acceleration = acceleration;
        this.maxVelocity = maxVelocity;
        this.currentPosition = currentPosition;
        this.targetPosition = targetPosition;
        this.currentVelocity = 0.0;
    }

    /**
     * Updates the current position based on the target position and the current velocity.
     *
     * @return the current position in units
     */
    public double update() {
        double timeStep;

        // Reset the timer the first time the motion profile is run
        if (!hasRun) {
            hasRun = true;
            timer.reset();
            timeStep = DEFAULT_TIMESTEP;
        } else {
            timeStep = timer.seconds();
        }

        double distanceToTarget = targetPosition - currentPosition;
        double direction = Math.signum(distanceToTarget);

        // Calculate the distance required to stop
        double stoppingDistance = Math.pow(currentVelocity, 2) / (2 * acceleration);

        if (Math.abs(distanceToTarget) <= stoppingDistance) {
            // Deceleration phase
            currentVelocity -= direction * acceleration * timeStep;
        } else if (Math.abs(currentVelocity) < maxVelocity) {
            // Acceleration phase
            currentVelocity += direction * acceleration * timeStep;
        } else {
            // Constant velocity phase
            currentVelocity = direction * maxVelocity;
        }

        // Update the current position
        currentPosition += currentVelocity * timeStep;

        // Reset the timer so we can measure the next loop time
        timer.reset();

        return currentPosition;
    }

    /**
     * Gets the current position of the profile.
     *
     * @return the current position in units
     */
    public double getCurrentPosition() {
        return currentPosition;
    }

    /**
     * Determines if the final target position has been reached by the motion profile.
     *
     * @return the current velocity in units per second
     */
    public boolean isAtTarget() {
        return Math.abs(targetPosition - currentPosition) < TOLERABLE_POSITION_ERROR && Math.abs(currentVelocity) < TOLERABLE_VELOCITY_ERROR;
    }
}
