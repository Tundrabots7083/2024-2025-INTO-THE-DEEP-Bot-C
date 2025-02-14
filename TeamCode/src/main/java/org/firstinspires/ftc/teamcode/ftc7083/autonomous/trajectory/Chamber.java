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
public class Chamber {
    // Initial pose for the robot
    public static double INITIAL_POSE_X = 22.5;
    public static double INITIAL_POSE_Y = -60.0;
    public static double INITIAL_POSE_ORIENTATION = 90.0;

    // Timeout for moving the intake to the wall
    public static double INTAKE_SPECIMEN_WALL_TIMEOUT = 0.4;

    // Position for scoring on the high chamber
    public static double CHAMBER_PRELOAD_X = 0;
    public static double CHAMBER_PRELOAD_Y = -46;
    public static double CHAMBER_PRELOAD_ORIENTATION = 120;
    public static double CHAMBER_PRELOAD_SCORE_Y = -44.5;

    // Positions for being between the chamber and the spike marks
    public static double SPIKE_MARK_1_SIDE_OF_CHAMBER_X = 30;
    public static double SPIKE_MARK_1_REVERSE_FROM_CHAMBER_Y = -48;
    public static double SPIKE_MARK_1_SIDE_OF_CHAMBER_ORIENTATION = 15;

    // Positions to push specimen from spike mark 1 to the observation zone
    public static double SPIKE_MARK_1_Y = -5;
    public static double SPIKE_MARK_1_ORIENTATION = -65;
    public static double SPIKE_MARK_1_X = 18;
    public static double SPIKE_MARK_1_OBSERVATION_ZONE_Y = -38;
    public static double SPIKE_MARK_1_OBSERVATION_ZONE_ORIENTATION = -65;

    // Pickup specimen 1 from wall
    public static double INTAKE_SPECIMEN_1_AT_WALL_STAGE_1_Y = -43.0;
    public static double INTAKE_SPECIMEN_1_AT_WALL_STAGE_1_ORIENTATION = -65;
    public static double INTAKE_SPECIMEN_1_AT_WALL_STAGE_2_Y = INTAKE_SPECIMEN_1_AT_WALL_STAGE_1_Y - 2.5;
    public static double INTAKE_SPECIMEN_1_AT_WALL_STAGE_2_ORIENTATION = -60;

    // Position for scoring specimen 2 on the high chamber
    public static double CHAMBER_SPECIMEN_1_X = CHAMBER_PRELOAD_X;
    public static double CHAMBER_SPECIMEN_1_Y = -48;
    public static double CHAMBER_SPECIMEN_1_ORIENTATION = CHAMBER_PRELOAD_ORIENTATION;
    public static double CHAMBER_SPECIMEN_1_SCORE_Y = -46;

    // Pickup specimen 2 from wall
    public static double INTAKE_SPECIMEN_2_AWAY_FROM_WALL_X = 37.5;
    public static double INTAKE_SPECIMEN_2_AWAY_FROM_WALL_Y = -38;
    public static double INTAKE_SPECIMEN_2_AWAY_FROM_WALL_ORIENTATION = -200;
    public static double INTAKE_SPECIMEN_2_AT_WALL_STAGE_1_Y = -43.0;
    public static double INTAKE_SPECIMEN_2_AT_WALL_STAGE_1_ORIENTATION = -65;
    public static double INTAKE_SPECIMEN_2_AT_WALL_STAGE_2_Y = INTAKE_SPECIMEN_2_AT_WALL_STAGE_1_Y - 2.5;
    public static double INTAKE_SPECIMEN_2_AT_WALL_STAGE_2_ORIENTATION = -60;

    // Position for scoring specimen 2 on the high chamber
    public static double CHAMBER_SPECIMEN_2_X = CHAMBER_PRELOAD_X;
    public static double CHAMBER_SPECIMEN_2_Y = -48;
    public static double CHAMBER_SPECIMEN_2_ORIENTATION = CHAMBER_PRELOAD_ORIENTATION;
    public static double CHAMBER_SPECIMEN_2_SCORE_Y = -46;

    // Park in the observation zone
    public static double PARK_X = 55;
    public static double PARK_Y = -40;
    public static double PARK_ORIENTATION = 120;

    private final TrajectoryActionBuilder actionBuilder;

    /**
     * Creates a new autonomous trajectory builder for the Red Alliance when scoring on the
     * chamber. This uses the default Pose2d for the robot.
     *
     * @param drive the Mecanum Drive used to move the robot autonomously
     */
    public Chamber(AutoMecanumDrive drive) {
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
    public Chamber(AutoMecanumDrive drive, Pose2d initialPose) {
        this(drive.actionBuilder(initialPose));
    }

    /**
     * Creates a new autonomous trajectory builder for the Red Alliance when scoring on the
     * chamber. This uses the specified trajectory action builder for building the trajectories.
     *
     * @param actionBuilder the action builder to use when creating the trajectories
     */
    public Chamber(TrajectoryActionBuilder actionBuilder) {
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

                // Move the the spike marks, push the sample on spike mark 1 to the observation zone
                .stopAndAdd(ias.actionMoveToIntakeSpecimenOffWallPosition())
                .lineToY(SPIKE_MARK_1_REVERSE_FROM_CHAMBER_Y)
                .strafeToSplineHeading(new Vector2d(SPIKE_MARK_1_SIDE_OF_CHAMBER_X, SPIKE_MARK_1_REVERSE_FROM_CHAMBER_Y), Math.toRadians(SPIKE_MARK_1_SIDE_OF_CHAMBER_ORIENTATION))
                .strafeToSplineHeading(new Vector2d(SPIKE_MARK_1_X, SPIKE_MARK_1_Y), Math.toRadians(SPIKE_MARK_1_ORIENTATION))
                .lineToYLinearHeading(SPIKE_MARK_1_OBSERVATION_ZONE_Y, Math.toRadians(SPIKE_MARK_1_OBSERVATION_ZONE_ORIENTATION))

                // Move to the wall and pickup specimen 1
                .lineToYLinearHeading(INTAKE_SPECIMEN_1_AT_WALL_STAGE_1_Y, Math.toRadians(INTAKE_SPECIMEN_1_AT_WALL_STAGE_1_ORIENTATION))
                .waitSeconds(INTAKE_SPECIMEN_WALL_TIMEOUT)
                .lineToYLinearHeading(INTAKE_SPECIMEN_1_AT_WALL_STAGE_2_Y, Math.toRadians(INTAKE_SPECIMEN_1_AT_WALL_STAGE_2_ORIENTATION))
                .stopAndAdd(ias.actionIntakeSpecimenFromWall())

                // Move to the chamber and score specimen 1
                .strafeToSplineHeading(new Vector2d(CHAMBER_SPECIMEN_1_X, CHAMBER_SPECIMEN_1_Y), Math.toRadians(CHAMBER_SPECIMEN_1_ORIENTATION))
                .stopAndAdd(ias.actionMoveToScoreSpecimenHighChamber())
                .lineToY(CHAMBER_SPECIMEN_1_SCORE_Y)
                .stopAndAdd(ias.actionScoreSpecimenHighChamber())

                // Move to the observation zone to pickup specimen 2 from the wall and pickup the specimen
                .strafeTo(new Vector2d(INTAKE_SPECIMEN_2_AWAY_FROM_WALL_X, INTAKE_SPECIMEN_2_AWAY_FROM_WALL_Y))
                .stopAndAdd(ias.actionMoveToIntakeSpecimenOffWallPosition())
                .turn(Math.toRadians(INTAKE_SPECIMEN_2_AWAY_FROM_WALL_ORIENTATION))
                .stopAndAdd(ias.actionMoveToIntakeSpecimenOffWallPosition())
                .lineToYLinearHeading(INTAKE_SPECIMEN_2_AT_WALL_STAGE_1_Y, Math.toRadians(INTAKE_SPECIMEN_2_AT_WALL_STAGE_1_ORIENTATION))
                .waitSeconds(INTAKE_SPECIMEN_WALL_TIMEOUT)
                .lineToYLinearHeading(INTAKE_SPECIMEN_2_AT_WALL_STAGE_2_Y, Math.toRadians(INTAKE_SPECIMEN_2_AT_WALL_STAGE_2_ORIENTATION))
                .stopAndAdd(ias.actionIntakeSpecimenFromWall())

                // Move to the chamber and score specimen 2
                .strafeToSplineHeading(new Vector2d(CHAMBER_SPECIMEN_2_X, CHAMBER_SPECIMEN_2_Y), Math.toRadians(CHAMBER_SPECIMEN_2_ORIENTATION))
                .stopAndAdd(ias.actionMoveToScoreSpecimenHighChamber())
                .lineToY(CHAMBER_SPECIMEN_2_SCORE_Y)
                .stopAndAdd(ias.actionScoreSpecimenHighChamber())

                // Park the robot
                .strafeToSplineHeading(new Vector2d(PARK_X, PARK_Y), Math.toRadians(PARK_ORIENTATION))
                .stopAndAdd(ias.actionMoveToStartPosition())
                .build();
    }
}
