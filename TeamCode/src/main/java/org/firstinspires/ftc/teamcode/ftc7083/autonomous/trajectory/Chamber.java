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
 * Autonomous trajectory builder for scoring specimen on the high chamber.
 */
@Config
public class Chamber {
    // Timeout for moving the intake to the wall
    public static double INTAKE_SPECIMEN_WALL_TIMEOUT = 0.3;

    // Initial pose for the robot
    public static double INITIAL_POSE_X = 22.5;
    public static double INITIAL_POSE_Y = -60.0;
    public static double INITIAL_POSE_ORIENTATION = 90.0;

    // Position for scoring the preloaded specimen on the high chamber
    public static double SPECIMEN_1_CHAMBER_X = -4;
    public static double SPECIMEN_1_CHAMBER_Y = -49;
    public static double SPECIMEN_1_CHAMBER_ORIENTATION = 120;
    public static double SPECIMEN_1_SCORE_CHAMBER_Y = -47.5;

    // Positions for strafing the sample from spike mark 1 into the observation zone
    public static double SPIKE_MARK_1_REVERSE_FROM_CHAMBER_Y = -48;
    public static double SPIKE_MARK_1_SIDE_OF_CHAMBER_X = 30;
    public static double SPIKE_MARK_1_SIDE_OF_CHAMBER_ORIENTATION = 15;
    public static double SPIKE_MARK_1_Y = -5;
    public static double SPIKE_MARK_1_ORIENTATION = -55;
    public static double SPIKE_MARK_1_X = 18;
    public static double SPIKE_MARK_1_OBSERVATION_ZONE_X = 40;
    public static double SPIKE_MARK_1_OBSERVATION_ZONE_Y = -41;
    public static double SPIKE_MARK_1_OBSERVATION_ZONE_ORIENTATION = -65;

    // Pickup specimen 2 from wall
    public static double SPECIMEN_2_INTAKE_AT_WALL_STAGE_1_Y = -43;
    public static double SPECIMEN_2_INTAKE_AT_WALL_STAGE_1_ORIENTATION = -65;

    // Position for scoring specimen 2 on the high chamber
    public static double SPECIMEN_2_CHAMBER_X = -2;
    public static double SPECIMEN_2_CHAMBER_Y = -50.5;
    public static double SPECIMEN_2_CHAMBER_ORIENTATION = SPECIMEN_1_CHAMBER_ORIENTATION;
    public static double SPECIMEN_2_CHAMBER_SCORE_Y = -48.5;

    // Pickup specimen 3 from wall
    public static double SPECIMEN_3_REVERSE_FROM_CHAMBER_Y = -48;
    public static double SPECIMEN_3_INTAKE_AWAY_FROM_WALL_X = 37.5;
    public static double SPECIMEN_3_INTAKE_AWAY_FROM_WALL_Y = -38;
    public static double SPECIMEN_3_INTAKE_AWAY_FROM_WALL_ORIENTATION = -50;
    public static double SPECIMEN_3_INTAKE_AT_WALL_STAGE_1_Y = -43.0;
    public static double SPECIMEN_3_INTAKE_AT_WALL_STAGE_1_ORIENTATION = -50;
    public static double SPECIMEN_3_INTAKE_AT_WALL_STAGE_2_Y = SPECIMEN_3_INTAKE_AT_WALL_STAGE_1_Y - 2.5;
    public static double SPECIMEN_3_INTAKE_AT_WALL_STAGE_2_ORIENTATION = -45;

    // Position for scoring specimen 3 on the high chamber
    public static double SPECIMEN_3_CHAMBER_X = 0;
    public static double SPECIMEN_3_CHAMBER_Y = -49;
    public static double SPECIMEN_3_CHAMBER_ORIENTATION = 135;
    public static double SPECIMEN_3_CHAMBER_SCORE_Y = -47;

    // Park in the observation zone
    public static double PARK_X = 50;
    public static double PARK_Y = -35;
    public static double PARK_ORIENTATION = 140;

    private final TrajectoryActionBuilder actionBuilder;

    /**
     * Creates a new autonomous trajectory builder for scoring on the high chamber.
     * This uses the default Pose2d for the robot.
     *
     * @param drive the Mecanum Drive used to move the robot autonomously
     */
    public Chamber(AutoMecanumDrive drive) {
        this(drive, new Pose2d(new Vector2d(INITIAL_POSE_X, INITIAL_POSE_Y), Math.toRadians(INITIAL_POSE_ORIENTATION)));
    }

    /**
     * Creates a new autonomous trajectory builder for scoring on the high chamber.
     * This uses the specified pose for the robot, which allows the invoker to override
     * the pose if desired.
     *
     * @param drive       the Mecanum Drive used to move the robot autonomously
     * @param initialPose the initial pose for the robot
     */
    public Chamber(AutoMecanumDrive drive, Pose2d initialPose) {
        this(drive.actionBuilder(initialPose));
    }

    /**
     * Creates a new autonomous trajectory builder for scoring on the high chamber.
     * This uses the specified trajectory action builder for building the trajectories.
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
                .strafeToSplineHeading(new Vector2d(SPECIMEN_1_CHAMBER_X, SPECIMEN_1_CHAMBER_Y), Math.toRadians(SPECIMEN_1_CHAMBER_ORIENTATION))
                .stopAndAdd(ias.actionMoveToScoreSpecimenHighChamber())
                .lineToY(SPECIMEN_1_SCORE_CHAMBER_Y)
                .stopAndAdd(ias.actionScoreSpecimenHighChamber())

                // Move the the spike marks, push the sample on spike mark 1 to the observation zone
                .lineToY(SPIKE_MARK_1_REVERSE_FROM_CHAMBER_Y)
                .strafeToSplineHeading(new Vector2d(SPIKE_MARK_1_SIDE_OF_CHAMBER_X, SPIKE_MARK_1_REVERSE_FROM_CHAMBER_Y), Math.toRadians(SPIKE_MARK_1_SIDE_OF_CHAMBER_ORIENTATION))
                .stopAndAdd(ias.actionMoveToIntakeSpecimenOffWallPosition())
                .strafeToSplineHeading(new Vector2d(SPIKE_MARK_1_X, SPIKE_MARK_1_Y), Math.toRadians(SPIKE_MARK_1_ORIENTATION))
                .strafeToSplineHeading(new Vector2d(SPIKE_MARK_1_OBSERVATION_ZONE_X, SPIKE_MARK_1_OBSERVATION_ZONE_Y), Math.toRadians(SPIKE_MARK_1_OBSERVATION_ZONE_ORIENTATION))

                // Move to the wall and pickup specimen 2
                .waitSeconds(INTAKE_SPECIMEN_WALL_TIMEOUT)
                .lineToYLinearHeading(SPECIMEN_2_INTAKE_AT_WALL_STAGE_1_Y, Math.toRadians(SPECIMEN_2_INTAKE_AT_WALL_STAGE_1_ORIENTATION))
                .stopAndAdd(ias.actionIntakeSpecimenFromWall())

                // Move to the chamber and score specimen 2
                .strafeToSplineHeading(new Vector2d(SPECIMEN_2_CHAMBER_X, SPECIMEN_2_CHAMBER_Y), Math.toRadians(SPECIMEN_2_CHAMBER_ORIENTATION))
                .stopAndAdd(ias.actionMoveToScoreSpecimenHighChamber())
                .lineToY(SPECIMEN_2_CHAMBER_SCORE_Y)
                .stopAndAdd(ias.actionScoreSpecimenHighChamber())

                // Move to the wall in preparation to pickup specimen 3
                .lineToY(SPECIMEN_3_REVERSE_FROM_CHAMBER_Y)
                .strafeToSplineHeading(new Vector2d(SPECIMEN_3_INTAKE_AWAY_FROM_WALL_X, SPECIMEN_3_INTAKE_AWAY_FROM_WALL_Y), Math.toRadians(SPECIMEN_3_INTAKE_AWAY_FROM_WALL_ORIENTATION))
                .stopAndAdd(ias.actionMoveToIntakeSpecimenOffWallPosition())

                // Move to the wall and pickup specimen 3
                .lineToYLinearHeading(SPECIMEN_3_INTAKE_AT_WALL_STAGE_1_Y, Math.toRadians(SPECIMEN_3_INTAKE_AT_WALL_STAGE_1_ORIENTATION))
//                .waitSeconds(INTAKE_SPECIMEN_WALL_TIMEOUT)
//                .lineToYLinearHeading(SPECIMEN_3_INTAKE_AT_WALL_STAGE_2_Y, Math.toRadians(SPECIMEN_3_INTAKE_AT_WALL_STAGE_2_ORIENTATION))
//                .stopAndAdd(ias.actionIntakeSpecimenFromWall())

//                // Move to the chamber and score specimen 3
//                .strafeToSplineHeading(new Vector2d(SPECIMEN_3_CHAMBER_X, SPECIMEN_3_CHAMBER_Y), Math.toRadians(SPECIMEN_3_CHAMBER_ORIENTATION))
//                .stopAndAdd(ias.actionMoveToScoreSpecimenHighChamber())
//                .lineToY(SPECIMEN_3_CHAMBER_SCORE_Y)
//                .stopAndAdd(ias.actionScoreSpecimenHighChamber())

                // Park the robot
//                .strafeToSplineHeading(new Vector2d(PARK_X, PARK_Y), Math.toRadians(PARK_ORIENTATION))
//                .stopAndAdd(ias.actionMoveToStartPosition())
                .waitSeconds(15)
                .build();
    }
}
