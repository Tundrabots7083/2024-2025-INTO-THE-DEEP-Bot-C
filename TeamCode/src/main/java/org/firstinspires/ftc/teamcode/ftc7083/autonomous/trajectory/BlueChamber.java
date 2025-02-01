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
public class BlueChamber {
    // Initial pose for the robot
    public static double INITIAL_POSE_X = 22.5;
    public static double INITIAL_POSE_Y = -60.0;
    public static double INITIAL_POSE_ORIENTATION = 90.0;

    // Position for scoring on the high chamber
    public static double CHAMBER_PRELOAD_X = 5.5;
    public static double CHAMBER_PRELOAD_Y = -42;
    public static double CHAMBER_PRELOAD_ORIENTATION = 95;
    public static double CHAMBER_PRELOAD_SCORE_Y = -42; // CHAMBER_PRELOAD_Y + 5.5;

    // Positions for being between the chamber and the spike marks
    public static double SPIKE_MARK_1_SIDE_OF_CHAMBER_X = 43.5;
    public static double SPIKE_MARK_1_SIDE_OF_CHAMBER_Y = -44.5;
    public static double SPIKE_MARK_1_SIDE_OF_CHAMBER_ORIENTATION = 95;

    // Positions for spike mark 1
    public static double SPIKE_MARK_1_Y = SPIKE_MARK_1_SIDE_OF_CHAMBER_Y + 10.0;
    public static double SPIKE_MARK_1_X = SPIKE_MARK_1_SIDE_OF_CHAMBER_X + 5.0;
    public static double SPIKE_MARK_1_OBSERVATION_ZONE_Y = SPIKE_MARK_1_Y - 20.0;

    // Positions for spike mark 2
    public static double SPIKE_MARK_2_Y = SPIKE_MARK_1_Y;
    public static double SPIKE_MARK_2_X = SPIKE_MARK_1_X + 5.0;
    public static double SPIKE_MARK_2_OBSERVATION_ZONE_Y = SPIKE_MARK_1_OBSERVATION_ZONE_Y;

    // Positions for spike mark 3
    public static double SPIKE_MARK_3_Y = SPIKE_MARK_1_Y;
    public static double SPIKE_MARK_3_X = SPIKE_MARK_1_X + 5.0;
    public static double SPIKE_MARK_3_OBSERVATION_ZONE_Y = SPIKE_MARK_1_OBSERVATION_ZONE_Y;

    // Pickup specimen from wall
    public static double INTAKE_SPECIMEN_X = 47;
    public static double INTAKE_SPECIMEN_Y = -50;

    // Position for scoring on the high chamber
    public static double CHAMBER_SPECIMEN_1_X = CHAMBER_PRELOAD_X;
    public static double CHAMBER_SPECIMEN_1_Y = CHAMBER_PRELOAD_Y;
    public static double CHAMBER_SPECIMEN_1_ORIENTATION = CHAMBER_PRELOAD_ORIENTATION;
    public static double CHAMBER_SPECIMEN_1_SCORE_Y = CHAMBER_SPECIMEN_1_Y + 6.5;

    // Park in the observation zone
    public static double PARK_X = 55;
    public static double PARK_Y = -54;
    public static double PARK_ORIENTATION = 120;

    private final TrajectoryActionBuilder actionBuilder;

    /**
     * Creates a new autonomous trajectory builder for the Red Alliance when scoring on the
     * chamber. This uses the default Pose2d for the robot.
     *
     * @param drive the Mecanum Drive used to move the robot autonomously
     */
    public BlueChamber(AutoMecanumDrive drive) {
        this(drive, new Pose2d(new Vector2d(INITIAL_POSE_X, INITIAL_POSE_Y), Math.toRadians(INITIAL_POSE_ORIENTATION)));
    }

    /**
     * Creates a new autonomous trajectory builder for the Red Alliance when scoring on the
     * chamber. This uses the specified pose for the robot, which allows the invoker to override
     * the pose if desired.
     *
     * @param drive       the Mecanum Drive used to move the robot autonomously
     * @param initialPose the initial pose for the robot
     */
    public BlueChamber(AutoMecanumDrive drive, Pose2d initialPose) {
        this(drive.actionBuilder(initialPose));
    }

    /**
     * Creates a new autonomous trajectory builder for the Red Alliance when scoring on the
     * chamber. This uses the specified trajectory action builder for building the trajectories.
     *
     * @param actionBuilder the action builder to use when creating the trajectories
     */
    public BlueChamber(TrajectoryActionBuilder actionBuilder) {
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
                .strafeToSplineHeading(new Vector2d(CHAMBER_PRELOAD_X, CHAMBER_PRELOAD_Y), Math.toRadians(CHAMBER_PRELOAD_ORIENTATION))
                .stopAndAdd(ias.actionMoveToScoreSpecimenHighChamber())
                .lineToY(CHAMBER_PRELOAD_SCORE_Y)
                .stopAndAdd(ias.actionScoreSpecimenHighChamber())
//                // Move from in front of the chamber to the side where the spike marks are located
//                .strafeToSplineHeading(new Vector2d(SPIKE_MARK_1_SIDE_OF_CHAMBER_X, SPIKE_MARK_1_SIDE_OF_CHAMBER_Y), Math.toRadians(SPIKE_MARK_1_SIDE_OF_CHAMBER_ORIENTATION))
//                // Move the the spike marks and strafe the sample on spike mark 1 to the observation zone
//                .lineToY(SPIKE_MARK_1_Y)
//                .lineToX(SPIKE_MARK_1_X)
//                .lineToY(SPIKE_MARK_1_OBSERVATION_ZONE_Y)
//                // Move the the spike marks and strafe the sample on spike mark 2 to the observation zone
//                .lineToY(SPIKE_MARK_2_Y)
//                .lineToX(SPIKE_MARK_2_X)
//                .lineToY(SPIKE_MARK_2_OBSERVATION_ZONE_Y)
//                // Move the the spike marks and strafe the sample on spike mark 2 to the observation zone
//                .lineToY(SPIKE_MARK_3_Y)
//                .lineToX(SPIKE_MARK_3_X)
//                .lineToY(SPIKE_MARK_3_OBSERVATION_ZONE_Y)
//                // Move to the observation zone to pickup specimen 1 from the wall and pickup the specimen
//                .stopAndAdd(ias.actionMoveToIntakeSpecimenOffWallPosition())
//                .strafeToSplineHeading(new Vector2d(INTAKE_SPECIMEN_X, INTAKE_SPECIMEN_Y), Math.toRadians(SPIKE_MARK_1_SIDE_OF_CHAMBER_ORIENTATION))
//                .stopAndAdd(ias.actionIntakeSpecimenFromWall())
//                // Move to the chamber and score the specimen
//                .strafeToSplineHeading(new Vector2d(CHAMBER_SPECIMEN_1_X, CHAMBER_SPECIMEN_1_Y), Math.toRadians(CHAMBER_SPECIMEN_1_ORIENTATION))
//                .stopAndAdd(ias.actionMoveToScoreSpecimenHighChamber())
//                .lineToY(CHAMBER_SPECIMEN_1_SCORE_Y)
//                .stopAndAdd(ias.actionScoreSpecimenHighChamber())
                // Park the robot
                .strafeToSplineHeading(new Vector2d(PARK_X, PARK_Y), Math.toRadians(PARK_ORIENTATION))
                .stopAndAdd(ias.actionMoveToStartPosition())
                .build();
    }
}
