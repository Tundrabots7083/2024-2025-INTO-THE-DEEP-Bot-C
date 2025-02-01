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
 * Autonomous trajectory builder for the Red Alliance when scoring specimen on the chamber.
 */
@Config
public class RedBasketSample {
    // Initial pose for the robot
    public static double INITIAL_POSE_X = -22.5;
    public static double INITIAL_POSE_Y = -60.0;
    public static double INITIAL_ORIENTATION = 90.0;

    // Position for scoring in the high basket
    public static double BASKET_HIGH_PRELOAD_X = -58.5;
    public static double BASKET_HIGH_PRELOAD_Y = -58.5;;
    public static double BASKET_HIGH_PRELOAD_ORIENTATION = 65;

    // Intake positions for spike mark 1
    public static double YELLOW_SPIKE_MARK_1_X = -57.5;
    public static double YELLOW_SPIKE_MARK_1_Y = -52.5;
    public static double YELLOW_SPIKE_MARK_1_ORIENTATION = 108;

    // Position for scoring in the high basket
    public static double BASKET_HIGH_SPECIMEN_1_X = -58.5;
    public static double BASKET_HIGH_SPECIMEN_1_Y = -60.5;;
    public static double BASKET_HIGH_SPECIMEN_1_ORIENTATION = 65;

    // Intake positions for spike mark 2
    public static double YELLOW_SPIKE_MARK_2_X = -67.5;
    public static double YELLOW_SPIKE_MARK_2_Y = -55.5;
    public static double YELLOW_SPIKE_MARK_2_ORIENTATION = 110;

    // Position for scoring in the high basket
    public static double BASKET_HIGH_SPECIMEN_2_X = -62.5;
    public static double BASKET_HIGH_SPECIMEN_2_Y = -62.5;;
    public static double BASKET_HIGH_SPECIMEN_2_ORIENTATION = 65;

    private final TrajectoryActionBuilder actionBuilder;

    /**
     * Creates a new autonomous trajectory builder for the Red Alliance when scoring on the
     * chamber. This uses the default Pose2d for the robot.
     *
     * @param drive the Mecanum Drive used to move the robot autonomously
     */
    public RedBasketSample(AutoMecanumDrive drive) {
        this(drive, new Pose2d(new Vector2d(INITIAL_POSE_X, INITIAL_POSE_Y), INITIAL_ORIENTATION));
    }

    /**
     * Creates a new autonomous trajectory builder for the Red Alliance when scoring on the
     * chamber. This uses the specified pose for the robot, which allows the invoker to override
     * the pose if desired.
     *
     * @param drive       the Mecanum Drive used to move the robot autonomously
     * @param initialPose the initial pose for the robot
     */
    public RedBasketSample(AutoMecanumDrive drive, Pose2d initialPose) {
        this(drive.actionBuilder(initialPose));
    }

    /**
     * Creates a new autonomous trajectory builder for the Red Alliance when scoring on the
     * chamber. This uses the specified trajectory action builder for building the trajectories.
     *
     * @param actionBuilder the action builder to use when creating the trajectories
     */
    public RedBasketSample(TrajectoryActionBuilder actionBuilder) {
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
                .strafeToSplineHeading(new Vector2d(BASKET_HIGH_PRELOAD_X, BASKET_HIGH_PRELOAD_Y), Math.toRadians(BASKET_HIGH_PRELOAD_ORIENTATION))
                .stopAndAdd(ias.actionScoreSampleHighBasket())
                // Pick up the sample from Spike Mark 1 and score in the high basket
                .strafeToSplineHeading(new Vector2d(YELLOW_SPIKE_MARK_1_X, YELLOW_SPIKE_MARK_1_Y), Math.toRadians(YELLOW_SPIKE_MARK_1_ORIENTATION))
                .stopAndAdd(ias.actionIntakeSampleFromSpikeMark())
                .strafeToSplineHeading(new Vector2d(BASKET_HIGH_SPECIMEN_1_X, BASKET_HIGH_SPECIMEN_1_Y), Math.toRadians(BASKET_HIGH_SPECIMEN_1_ORIENTATION))
                .stopAndAdd(ias.actionScoreSampleHighBasket())
                // Pick up the sample from Spike Mark 2 and score in the high basket
                .strafeToSplineHeading(new Vector2d(YELLOW_SPIKE_MARK_2_X, YELLOW_SPIKE_MARK_2_Y), Math.toRadians(YELLOW_SPIKE_MARK_2_ORIENTATION))
                .stopAndAdd(ias.actionIntakeSampleFromSpikeMark())
                .strafeToSplineHeading(new Vector2d(BASKET_HIGH_SPECIMEN_2_X, BASKET_HIGH_SPECIMEN_2_Y), Math.toRadians(BASKET_HIGH_SPECIMEN_2_ORIENTATION))
                .stopAndAdd(ias.actionScoreSampleHighBasket())
                // Move to the start position
                .stopAndAdd(ias.actionMoveToStartPosition())
                .build();
    }
}
