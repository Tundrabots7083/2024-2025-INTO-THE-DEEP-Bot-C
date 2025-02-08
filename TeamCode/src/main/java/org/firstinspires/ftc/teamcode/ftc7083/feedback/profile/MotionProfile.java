package org.firstinspires.ftc.teamcode.ftc7083.feedback.profile;

import androidx.annotation.NonNull;

/**
 * The MotionProfile class represents a motion profile with specified maximum acceleration, maximum velocity,
 * start position, and end position. It calculates the current position based on the elapsed time since the
 * profile started.
 */
public class MotionProfile {
    private final double maxAcceleration;
    private final double maxVelocity;
    private final double startPosition;
    private double endPosition;
    private long startTime;

    /**
     * Creates a new MotionProfile with the specified maximum acceleration, maximum velocity, start position, and end position.
     * The distance unit used does not matter as long as it is consistent. It can be encoder ticks, inches, feet, etc.
     * The time unit component of the maximum acceleration and maximum velocity constants must be seconds.
     *
     * @param maxAcceleration The maximum acceleration of the motion profile in units per second squared.
     * @param maxVelocity     The maximum velocity of the motion profile in units per second.
     * @param startPosition   The start position of the motion profile in units.
     * @param endPosition     The end position of the motion profile in units.
     */
    public MotionProfile(double maxAcceleration, double maxVelocity, double startPosition, double endPosition) {
        this.maxAcceleration = maxAcceleration;
        this.maxVelocity = maxVelocity;
        this.startPosition = startPosition;
        this.endPosition = endPosition;

        if (this.startPosition == this.endPosition) {
            this.endPosition += 1;
        }
    }

    /**
     * Returns the elapsed time as the profile calculates it
     *
     * @return elapsed time
     */
    public long getTimestamp() {
        return (long)((System.currentTimeMillis() - this.startTime)); // leave as milliseconds
    }

    /**
     * Returns indication as to whether the motion profile has reached it's end.
     *
     * @return <code>true</code> if the motion profile has reached it's end;
     * <code>false</code> if there is still some motion required to reach the end.
     */
    public boolean isAtEnd() {
        double elapsedTime = (System.currentTimeMillis() - this.startTime) / 1000.0; // convert to seconds
        double distance = Math.abs(endPosition - startPosition);
        double accelerationDt = maxVelocity / maxAcceleration;
        double halfwayDistance = distance / 2;
        double accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);

        if (accelerationDistance > halfwayDistance) {
            accelerationDt = Math.sqrt(halfwayDistance / (0.5 * maxAcceleration));
        }

        double localMaxVelocity = maxAcceleration * accelerationDt;
        double decelerationDt = accelerationDt;
        double cruiseDistance = distance - 2 * accelerationDistance;
        double cruiseDt = cruiseDistance / localMaxVelocity;
        double entireDt = accelerationDt + cruiseDt + decelerationDt;

        return elapsedTime > entireDt;
    }

    /**
     * Calculates the current position based on the elapsed time since the motion profile started.
     *
     * @return The current position.
     */
    public double calculatePosition() {
        if (this.startTime == 0) {
            this.startTime = System.currentTimeMillis();
        }

        double elapsedTime = (System.currentTimeMillis() - this.startTime) / 1000.0; // convert to seconds
        double distance = Math.abs(endPosition - startPosition);
        double direction = Math.signum(endPosition - startPosition);
        double accelerationDt = maxVelocity / maxAcceleration;
        double halfwayDistance = distance / 2;
        double accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);

        if (accelerationDistance > halfwayDistance) {
            accelerationDt = Math.sqrt(halfwayDistance / (0.5 * maxAcceleration));
        }

        accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);
        double localMaxVelocity = maxAcceleration * accelerationDt;
        double decelerationDt = accelerationDt;
        double cruiseDistance = distance - 2 * accelerationDistance;
        double cruiseDt = cruiseDistance / localMaxVelocity;
        double decelerationTime = accelerationDt + cruiseDt;
        double entireDt = accelerationDt + cruiseDt + decelerationDt;

        double position;
        if (elapsedTime > entireDt) {
            // We've reached the end of the motion profile
            position = distance;
        } else if (elapsedTime < accelerationDt) {
            // Acceleration Phase
            position = 0.5 * maxAcceleration * Math.pow(elapsedTime, 2);
        } else if (elapsedTime < decelerationTime) {
            // Cruise Phase
            accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);
            double cruiseCurrentDt = elapsedTime - accelerationDt;
            position = accelerationDistance + localMaxVelocity * cruiseCurrentDt;
        } else {
            // Deceleration Phase
            accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);
            cruiseDistance = localMaxVelocity * cruiseDt;
            double decelerationCurrentTime = elapsedTime - decelerationTime;
            position = accelerationDistance + cruiseDistance + localMaxVelocity * decelerationCurrentTime - 0.5 * maxAcceleration * Math.pow(decelerationCurrentTime, 2);
        }

        return startPosition + direction * position;
    }


    /**
     * Gets a string representation of this motion profile.
     *
     * @return a string representation of this motion profile
     */
    @NonNull
    @Override
    public String toString() {
        return "MotionProfile{" +
                "maxAcceleration=" + maxAcceleration +
                ", maxVelocity=" + maxVelocity +
                ", startPosition=" + startPosition +
                ", endPosition=" + endPosition +
                ", startTime=" + startTime +
                '}';
    }
}