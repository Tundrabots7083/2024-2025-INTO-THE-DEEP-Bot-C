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
 * Autonomous trajectory builder for the Blue Alliance when scoring specimen on the chamber.
 */
@Config
public class BlueBasketSpecimen {
    // Initial pose for the robot
    public static double INITIAL_POSE_X = -RedBasketSpecimen.INITIAL_POSE_X;
    public static double INITIAL_POSE_Y = -RedBasketSpecimen.INITIAL_POSE_Y;
    public static double INITIAL_ORIENTATION = -RedBasketSpecimen.INITIAL_ORIENTATION;

    // Position for scoring on the high chamber
    public static double CHAMBER_HIGH_X = -RedBasketSpecimen.CHAMBER_HIGH_X;
    public static double CHAMBER_HIGH_Y = -RedBasketSpecimen.CHAMBER_HIGH_Y;
    public static double CHAMBER_HIGH_ORIENTATION = -RedBasketSpecimen.CHAMBER_HIGH_ORIENTATION;
    public static double HIGH_CHAMBER_DRIVE_FORWARD = -RedBasketSpecimen.HIGH_CHAMBER_DRIVE_FORWARD;

    // Intake positions for spike mark 1
    public static double YELLOW_SPIKE_MARK_1_X = -RedBasketSpecimen.YELLOW_SPIKE_MARK_1_X;
    public static double YELLOW_SPIKE_MARK_1_Y = -RedBasketSpecimen.YELLOW_SPIKE_MARK_1_Y;
    public static double YELLOW_SPIKE_MARK_1_ORIENTATION = -RedBasketSpecimen.YELLOW_SPIKE_MARK_1_ORIENTATION;

    // Position for scoring in the high basket
    public static double BASKET_HIGH_SPECIMEN_1_X = -RedBasketSpecimen.BASKET_HIGH_SPECIMEN_1_X;
    public static double BASKET_HIGH_SPECIMEN_1_Y = -RedBasketSpecimen.BASKET_HIGH_SPECIMEN_1_Y;;
    public static double BASKET_HIGH_SPECIMEN_1_ORIENTATION = -RedBasketSpecimen.BASKET_HIGH_SPECIMEN_1_ORIENTATION;

    // Intake positions for spike mark 2
    public static double YELLOW_SPIKE_MARK_2_X = -RedBasketSpecimen.YELLOW_SPIKE_MARK_2_X;
    public static double YELLOW_SPIKE_MARK_2_Y = -RedBasketSpecimen.YELLOW_SPIKE_MARK_2_Y;
    public static double YELLOW_SPIKE_MARK_2_ORIENTATION = -RedBasketSpecimen.YELLOW_SPIKE_MARK_2_ORIENTATION;

    // Position for scoring in the high basket
    public static double BASKET_HIGH_SPECIMEN_2_X = -RedBasketSpecimen.BASKET_HIGH_SPECIMEN_2_X;
    public static double BASKET_HIGH_SPECIMEN_2_Y = -RedBasketSpecimen.BASKET_HIGH_SPECIMEN_2_Y;;
    public static double BASKET_HIGH_SPECIMEN_2_ORIENTATION = -RedBasketSpecimen.BASKET_HIGH_SPECIMEN_2_ORIENTATION;

    private final TrajectoryActionBuilder actionBuilder;

    /**
     * Creates a new autonomous trajectory builder for the Blue Alliance when scoring on the
     * chamber. This uses the default Pose2d for the robot.
     *
     * @param drive the Mecanum Drive used to move the robot autonomously
     */
    public BlueBasketSpecimen(AutoMecanumDrive drive) {
        this(drive, new Pose2d(new Vector2d(INITIAL_POSE_X, INITIAL_POSE_Y), INITIAL_ORIENTATION));
    }

    /**
     * Creates a new autonomous trajectory builder for the Blue Alliance when scoring on the
     * chamber. This uses the specified pose for the robot, which allows the invoker to override
     * the pose if desired.
     *
     * @param drive       the Mecanum Drive used to move the robot autonomously
     * @param initialPose the initial pose for the robot
     */
    public BlueBasketSpecimen(AutoMecanumDrive drive, Pose2d initialPose) {
        this(drive.actionBuilder(initialPose));
    }

    /**
     * Creates a new autonomous trajectory builder for the Blue Alliance when scoring on the
     * chamber. This uses the specified trajectory action builder for building the trajectories.
     *
     * @param actionBuilder the action builder to use when creating the trajectories
     */
    public BlueBasketSpecimen(TrajectoryActionBuilder actionBuilder) {
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
                .strafeToSplineHeading(new Vector2d(CHAMBER_HIGH_X, CHAMBER_HIGH_Y), Math.toRadians(CHAMBER_HIGH_ORIENTATION))
                .stopAndAdd(ias.actionMoveToScoreSpecimenHighChamber())
                .lineToY(CHAMBER_HIGH_Y + HIGH_CHAMBER_DRIVE_FORWARD)
                .stopAndAdd(ias.actionScoreSpecimenHighChamber())
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
