package org.firstinspires.ftc.teamcode.ftc7083.subsystem.feedback;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.FeedForward;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Arm;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.LinearSlide;

/**
 * Feed forward component for the linear slide. This is used by the arm's PID controller to
 * compensate the pull of gravity on the linear slide, based on the angle of the arm.
 */
@Config
public class ArmFeedForward implements FeedForward {
    public static double LINEAR_SLIDE_DIVISOR = 20.0;
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

    /**
     * Calculate the feed forward value to add to the arm motor. This factors in the arm's
     * current angle, the length of the linear slide, and the gravity constant (kG).
     *
     * @param target the target position trying to be reached
     * @return the feed forward value to add to the arm motor power
     */
    @Override
    public double calculate(double target) {
        double radians = Math.toRadians(arm.getCurrentAngle());
        double power = Math.cos(radians) * kG;

        // A bit hacky to get the linear slide from the Robot class like this, but it works.
        // We really should find a cleaner way to do it, but that is left for the future.
        LinearSlide linearSlide = Robot.getInstance().linearSlide;
        power *= (1 + (linearSlide.getCurrentLength() / LINEAR_SLIDE_DIVISOR));

        return power;
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
