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
public class RedBasket {
    // Orientations for the robot, in degrees
    public static double ORIENTATION_CHAMBER = 90;
    public static double ORIENTATION_TOWARD_WALL = -90;
    public static double ORIENTATION_SPIKE_MARK_1 = 60;
    public static double ORIENTATION_SPIKE_MARK_2 = 90;
    public static double ORIENTATION_SPIKE_MARK_3 = 120;
    public static double ORIENTATION_BASKET = 45;
    public static double ORIENTATION_TOWARD_ASCENT_BARS = 0;

    // Initial pose for the robot
    public static double INITIAL_POSE_X = -16.0;
    public static double INITIAL_POSE_Y = -62.0;
    public static double INITIAL_HEADING_DEGREES = ORIENTATION_TOWARD_WALL;
    public static double INITIAL_HEADING = Math.toRadians(INITIAL_HEADING_DEGREES);

    // Position for scoring on the high chamber
    public static double CHAMBER_HIGH_X = -10;
    public static double CHAMBER_HIGH_Y = -35;

    // Position for scoring in the high basket
    public static double BASKET_HIGH_X = -52;
    public static double BASKET_HIGH_Y = -52;;

    // Intake positions for samples on the spike marks
    public static double YELLOW_SPIKE_MARK_X = -58;
    public static double YELLOW_SPIKE_MARK_Y = -45;

    // Park in the observation zone
    public static double PARK_APPROACH_X = -40;
    public static double PARK_X = -25;
    public static double PARK_Y = 0;

    private final TrajectoryActionBuilder actionBuilder;

    /**
     * Creates a new autonomous trajectory builder for the Red Alliance when scoring on the
     * chamber. This uses the default Pose2d for the robot.
     *
     * @param drive the Mecanum Drive used to move the robot autonomously
     */
    public RedBasket(AutoMecanumDrive drive) {
        this(drive, new Pose2d(new Vector2d(INITIAL_POSE_X, INITIAL_POSE_Y), INITIAL_HEADING));
    }

    /**
     * Creates a new autonomous trajectory builder for the Red Alliance when scoring on the
     * chamber. This uses the specified pose for the robot, which allows the invoker to override
     * the pose if desired.
     *
     * @param drive       the Mecanum Drive used to move the robot autonomously
     * @param initialPose the initial pose for the robot
     */
    public RedBasket(AutoMecanumDrive drive, Pose2d initialPose) {
        this(drive.actionBuilder(initialPose));
    }

    /**
     * Creates a new autonomous trajectory builder for the Red Alliance when scoring on the
     * chamber. This uses the specified trajectory action builder for building the trajectories.
     *
     * @param actionBuilder the action builder to use when creating the trajectories
     */
    public RedBasket(TrajectoryActionBuilder actionBuilder) {
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
                // Move to the chamber and score the specimen
                .strafeToSplineHeading(new Vector2d(CHAMBER_HIGH_X, CHAMBER_HIGH_Y), Math.toRadians(ORIENTATION_CHAMBER))
                .stopAndAdd(ias.actionScoreSpecimenHighChamber())
                // Pick up the sample from Spike Mark 1 and score in the high basket
                .strafeToSplineHeading(new Vector2d(YELLOW_SPIKE_MARK_X, YELLOW_SPIKE_MARK_Y), Math.toRadians(ORIENTATION_SPIKE_MARK_1))
                .stopAndAdd(ias.actionIntakeSampleFromSpikeMark())
                .strafeToSplineHeading(new Vector2d(BASKET_HIGH_X, BASKET_HIGH_Y), Math.toRadians(ORIENTATION_BASKET))
                .stopAndAdd(ias.actionScoreSampleHighBasket())
                // Pick up the sample from Spike Mark 2 and score in the high basket
                .strafeToSplineHeading(new Vector2d(YELLOW_SPIKE_MARK_X, YELLOW_SPIKE_MARK_Y), Math.toRadians(ORIENTATION_SPIKE_MARK_2))
                .stopAndAdd(ias.actionIntakeSampleFromSpikeMark())
                .strafeToSplineHeading(new Vector2d(BASKET_HIGH_X, BASKET_HIGH_Y), Math.toRadians(ORIENTATION_BASKET))
                .stopAndAdd(ias.actionScoreSampleHighBasket())
                // Pick up the sample from Spike Mark 2 and score in the high basket
                .strafeToSplineHeading(new Vector2d(YELLOW_SPIKE_MARK_X, YELLOW_SPIKE_MARK_Y), Math.toRadians(ORIENTATION_SPIKE_MARK_3))
                .stopAndAdd(ias.actionIntakeSampleFromSpikeMark())
                .strafeToSplineHeading(new Vector2d(BASKET_HIGH_X, BASKET_HIGH_Y), Math.toRadians(ORIENTATION_BASKET))
                .stopAndAdd(ias.actionScoreSampleHighBasket())
                // Park the robot
                .strafeToSplineHeading(new Vector2d(PARK_APPROACH_X, PARK_Y), Math.toRadians(ORIENTATION_TOWARD_ASCENT_BARS))
                .strafeTo(new Vector2d(PARK_X, PARK_Y))
                .stopAndAdd(ias.actionTouchAscentBarLow())
                .build();
    }
}
