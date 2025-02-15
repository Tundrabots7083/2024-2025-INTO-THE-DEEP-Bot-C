package org.firstinspires.ftc.teamcode.ftc7083.autonomous.trajectory;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;

/**
 * Autonomous trajectory builder for scoring samples in the baskets
 */
@Config
public class Basket {
    // Initial pose for the robot
    public static double INITIAL_POSE_X = -22.5;
    public static double INITIAL_POSE_Y = -60.0;
    public static double INITIAL_POSE_ORIENTATION = 90.0;

    // Position for scoring the pre-loaded sample in the high basket
    public static double SAMPLE_1_BASKET_X = -51.5;
    public static double SAMPLE_1_BASKET_Y = -64.5;;
    public static double SAMPLE_1_BASKET_ORIENTATION = 65;

    // Intake positions for spike mark 1
    public static double SPIKE_MARK_1_X = -50.5;
    public static double SPIKE_MARK_1_Y = -54;
    public static double SPIKE_MARK_1_ORIENTATION = 140;

    // Position for scoring sample 2 in the high basket
    public static double SAMPLE_2_BASKET_X = -50.5;
    public static double SAMPLE_2_BASKET_Y = -63.5;;
    public static double SAMPLE_2_BASKET_ORIENTATION = 45; // 65;

    // Intake positions for spike mark 2
    public static double SPIKE_MARK_2_X = -58.5;
    public static double SPIKE_MARK_2_Y = -57.5;
    public static double SPIKE_MARK_2_ORIENTATION = 150;

    // Position for scoring sample 3 in the high basket
    public static double SAMPLE_3_BASKET_X = -55.5;
    public static double SAMPLE_3_BASKET_Y = -63.5;;
    public static double SAMPLE_3_BASKET_ORIENTATION = 45;

    private final TrajectoryActionBuilder actionBuilder;

    /**
     * Creates a new autonomous trajectory builder for the Red Alliance when scoring on the
     * chamber. This uses the default Pose2d for the robot.
     *
     * @param drive the Mecanum Drive used to move the robot autonomously
     */
    public Basket(AutoMecanumDrive drive) {
        this(drive, new Pose2d(new Vector2d(INITIAL_POSE_X, INITIAL_POSE_Y), INITIAL_POSE_ORIENTATION));
    }

    /**
     * Creates a new autonomous trajectory builder for scoring samples in the high basket.
     * This uses the specified pose for the robot, which allows the invoker to override
     * the pose if desired.
     *
     * @param drive       the Mecanum Drive used to move the robot autonomously
     * @param initialPose the initial pose for the robot
     */
    public Basket(AutoMecanumDrive drive, Pose2d initialPose) {
        this(drive.actionBuilder(initialPose));
    }

    /**
     * Creates a new autonomous trajectory builder for scoring samples in the high basket.
     * This uses the specified trajectory action builder for building the trajectories.
     *
     * @param actionBuilder the action builder to use when creating the trajectories
     */
    public Basket(TrajectoryActionBuilder actionBuilder) {
        this.actionBuilder = actionBuilder;
    }

    /**
     * Gets the action for running the trajectory for the autonomous driving period.
     *
     * @return the action for running the trajectory for the autonomous driving period
     */
    public Action getTrajectory() {
        IntakeAndScoringSubsystem ias = Robot.getInstance().intakeAndScoringSubsystem;
        return actionBuilder
                // Score pre-loaded sample in the basket
                .strafeToSplineHeading(new Vector2d(SAMPLE_1_BASKET_X, SAMPLE_1_BASKET_Y), Math.toRadians(SAMPLE_1_BASKET_ORIENTATION))
                .stopAndAdd(ias.actionScoreSampleHighBasket())

                // Pick up the sample from Spike Mark 1
                .strafeToSplineHeading(new Vector2d(SPIKE_MARK_1_X, SPIKE_MARK_1_Y), Math.toRadians(SPIKE_MARK_1_ORIENTATION))
                .stopAndAdd(ias.actionIntakeSampleFromSpikeMark())

                // Score sample 2 in the basket
                .strafeToSplineHeading(new Vector2d(SAMPLE_2_BASKET_X, SAMPLE_2_BASKET_Y), Math.toRadians(SAMPLE_2_BASKET_ORIENTATION))
                .stopAndAdd(ias.actionScoreSampleHighBasket())

                // Pick up the sample from Spike Mark 2
                .strafeToSplineHeading(new Vector2d(SPIKE_MARK_2_X, SPIKE_MARK_2_Y), Math.toRadians(SPIKE_MARK_2_ORIENTATION))
                .stopAndAdd(ias.actionIntakeSampleFromSpikeMark())

                // Score sample 3 in the basket
                .strafeToSplineHeading(new Vector2d(SAMPLE_3_BASKET_X, SAMPLE_3_BASKET_Y), Math.toRadians(SAMPLE_3_BASKET_ORIENTATION))
                .stopAndAdd(ias.actionScoreSampleHighBasket())

                // Move to the start position
                .stopAndAdd(ias.actionMoveToStartPosition())
                .build();
    }
}
