package org.firstinspires.ftc.teamcode.ftc7083.subsystem.feedback;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.ftc7083.feedback.FeedForward;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Arm;

/**
 * Feed forward component for the linear slide. This is used by the arm's PID controller to
 * compensate the pull of gravity on the linear slide, based on the angle of the arm.
 */
public class ArmFeedForward implements FeedForward {
    private final Arm arm;
    private final double kG;

    /**
     * Instantiates a new feed forward component for the linear slide based on the arm.
     *
     * @param arm the arm for the intake subsystem
     * @param kG  the gravity component to use in calculating the feed forward component
     */
    public ArmFeedForward(Arm arm, double kG) {
        this.arm = arm;
        this.kG = kG;
    }

    @Override
    public double calculate(double target) {
        double radians = Math.toRadians(arm.getCurrentAngle());
        return Math.cos(radians) * kG;
    }

    /**
     * Returns a string representation for the linear slide feed forward function.
     *
     * @return a string representation for the linear slide feed forward function
     */
    @NonNull
    public String toString() {
        return "LinearSlideFeedForward{" +
                "ArmAngle=" + arm.getCurrentAngle() +
                ", kG=" + kG +
                "}";
    }
}
