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
public class RedChamber {
    public static double DEBUG_ROTATE_180_DEGREES = Math.toRadians(180);
    // Orientations for the robot, in degrees
    public static double ORIENTATION_CHAMBER = 90;
    public static double ORIENTATION_TOWARD_WALL = -90;
    public static double ORIENTATION_SPIKE_MARK_1 = 60;
    public static double ORIENTATION_SPIKE_MARK_2 = -90;
    public static double ORIENTATION_SPIKE_MARK_3 = 120;

    // Initial pose for the robot
    public static double INITIAL_POSE_X = 22.5;
    public static double INITIAL_POSE_Y = -60.0;
    public static double INITIAL_HEADING = 90.0;

    // Position for scoring on the high chamber
    public static double CHAMBER_HIGH_X = 0;
    public static double CHAMBER_HIGH_Y = -38;
    public static double HIGH_CHAMBER_DRIVE_FORWARD = 6.5;

    // Positions for the spike marks
    public static double RED_SPIKE_MARK_X = 43.5;
    public static double RED_SPIKE_MARK_Y = -44.5;

    // Pickup specimen from wall
    public static double OBSERVATION_ZONE_X = 47;
    public static double OBSERVATION_ZONE_Y = -50;

    // Park in the observation zone
    public static double PARK_X = 50;
    public static double PARK_Y = -60;

    private final TrajectoryActionBuilder actionBuilder;

    /**
     * Creates a new autonomous trajectory builder for the Red Alliance when scoring on the
     * chamber. This uses the default Pose2d for the robot.
     *
     * @param drive the Mecanum Drive used to move the robot autonomously
     */
    public RedChamber(AutoMecanumDrive drive) {
        this(drive, new Pose2d(new Vector2d(INITIAL_POSE_X, INITIAL_POSE_Y), Math.toRadians(INITIAL_HEADING)));
    }

    /**
     * Creates a new autonomous trajectory builder for the Red Alliance when scoring on the
     * chamber. This uses the specified pose for the robot, which allows the invoker to override
     * the pose if desired.
     *
     * @param drive       the Mecanum Drive used to move the robot autonomously
     * @param initialPose the initial pose for the robot
     */
    public RedChamber(AutoMecanumDrive drive, Pose2d initialPose) {
        this(drive.actionBuilder(initialPose));
    }

    /**
     * Creates a new autonomous trajectory builder for the Red Alliance when scoring on the
     * chamber. This uses the specified trajectory action builder for building the trajectories.
     *
     * @param actionBuilder the action builder to use when creating the trajectories
     */
    public RedChamber(TrajectoryActionBuilder actionBuilder) {
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
                .strafeTo(new Vector2d(CHAMBER_HIGH_X, CHAMBER_HIGH_Y))
                .stopAndAdd(ias.actionMoveToScoreSpecimenHighChamber())
                .lineToYConstantHeading(CHAMBER_HIGH_Y + HIGH_CHAMBER_DRIVE_FORWARD)
                .stopAndAdd(ias.actionScoreSpecimenHighChamber())
//                // Move the the spike marks and move the sample from Spike Mark 1 to the observation zone
//                .strafeTo(new Vector2d(RED_SPIKE_MARK_X, RED_SPIKE_MARK_Y))
//                .stopAndAdd(ias.actionIntakeSampleFromSpikeMark())
//                .turn(DEBUG_ROTATE_180_DEGREES)
//                .turnTo(ORIENTATION_TOWARD_WALL)
//                .stopAndAdd(ias.actionDropOffSample())
//                // Move the sample from Spike Mark 2 to the observation zone
//                .turnTo(Math.toRadians(ORIENTATION_SPIKE_MARK_2))
//                .stopAndAdd(ias.actionIntakeSampleFromSpikeMark())
//                .turnTo(Math.toRadians(ORIENTATION_TOWARD_WALL))
//                .stopAndAdd(ias.actionDropOffSample())
//                // Move the sample from Spike Mark 3 to the observation zone
//                .turnTo(Math.toRadians(ORIENTATION_SPIKE_MARK_3))
//                .stopAndAdd(ias.actionIntakeSampleFromSpikeMark())
//                .turnTo(Math.toRadians(ORIENTATION_TOWARD_WALL))
//                .stopAndAdd(ias.actionDropOffSample())
//                // Move to the observation zone to pickup specimen 1 from the wall and score on the chamber
//                .strafeToSplineHeading(new Vector2d(OBSERVATION_ZONE_X, OBSERVATION_ZONE_Y), Math.toRadians(ORIENTATION_TOWARD_WALL))
//                .stopAndAdd(ias.actionIntakeSpecimenFromWall())
//                .strafeToSplineHeading(new Vector2d(CHAMBER_HIGH_X, CHAMBER_HIGH_Y), Math.toRadians(ORIENTATION_CHAMBER))
//                .stopAndAdd(ias.actionScoreSpecimenHighChamber())
//                // Move to the observation zone to pickup specimen 2 from the wall and score on the chamber
//                .strafeToSplineHeading(new Vector2d(OBSERVATION_ZONE_X, OBSERVATION_ZONE_Y), Math.toRadians(ORIENTATION_TOWARD_WALL))
//                .stopAndAdd(ias.actionIntakeSpecimenFromWall())
//                .strafeToSplineHeading(new Vector2d(CHAMBER_HIGH_X, CHAMBER_HIGH_Y), Math.toRadians(ORIENTATION_CHAMBER))
//                .stopAndAdd(ias.actionScoreSpecimenHighChamber())
//                // Move to the observation zone to pickup specimen 3 from the wall and score on the chamber
//                .strafeToSplineHeading(new Vector2d(OBSERVATION_ZONE_X, OBSERVATION_ZONE_Y), Math.toRadians(ORIENTATION_TOWARD_WALL))
//                .stopAndAdd(ias.actionIntakeSpecimenFromWall())
//                .strafeToSplineHeading(new Vector2d(CHAMBER_HIGH_X, CHAMBER_HIGH_Y), Math.toRadians(ORIENTATION_CHAMBER))
//                .stopAndAdd(ias.actionScoreSpecimenHighChamber())
//                // Park the robot
//                .strafeTo(new Vector2d(PARK_X, PARK_Y))
//                .stopAndAdd(ias.actionMoveToStartPosition())
                .build();
    }
}
