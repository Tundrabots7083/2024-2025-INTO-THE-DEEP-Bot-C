package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionEx;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionExBase;
import org.firstinspires.ftc.teamcode.ftc7083.action.SequentialAction;
import org.firstinspires.ftc.teamcode.ftc7083.math.FTCMath;

/**
 * This class uses the Arm, LinearSlide, Wrist, and Claw subsystems to pick up, control,
 * and score samples and specimens.
 */
@Config
public class IntakeAndScoringSubsystem extends SubsystemBase {
    // Wait time for picking up samples
    public static int INTAKE_WAIT_TIME = 500;

    // Heights of scoring places for game are in inches
    public static double HIGH_CHAMBER_HEIGHT = 26.0;
    public static double HIGH_BASKET_HEIGHT = 43.0;
    public static double LOW_BASKET_HEIGHT = 25.75;
    public static double LOW_ASCENT_BAR_HEIGHT = 20.0;

    // Maximum horizontal length of robot when extended
    public static double MAX_EXTENDED_ROBOT_LENGTH = 40.0;

    // Robot measurements
    // Length of the arm and the wrist with zero extension in inches.
    public static double ARM_LENGTH = 15.0;
    // Height from the field to the center of rotation of the arm in inches.
    public static double ARM_HEIGHT = 9.5;

    // Arm Movement Constants
    public static double INTAKE_HEIGHT_ABOVE = ARM_HEIGHT - 2.6;                        // 6.9
    public static double INTAKE_HEIGHT_LOWERED = INTAKE_HEIGHT_ABOVE - 2.0;             // 4.9

    // (x, y) distances in inches from the center of rotation of the arm that the
    // scoring subsystem needs to reach to intake specimens and samples.
    public static double DEPOSIT_SAMPLE_X = ARM_LENGTH + 7.5;                           // 22.5
    public static double DEPOSIT_SAMPLE_Y = INTAKE_HEIGHT_LOWERED;                      // 4.9
    public static double INTAKE_CLOSE_ABOVE_X = ARM_LENGTH + 7.5;                       // 22.5
    public static double INTAKE_CLOSE_ABOVE_Y = INTAKE_HEIGHT_ABOVE - 2.0;              // 4.9
    public static double INTAKE_CLOSE_LOWERED_X = INTAKE_CLOSE_ABOVE_X;                 // 22.5
    public static double INTAKE_CLOSE_LOWERED_Y = INTAKE_HEIGHT_LOWERED;                // 4.9
    public static double INTAKE_FAR_ABOVE_X = ARM_LENGTH + 15.0;                        // 30.0
    public static double INTAKE_FAR_ABOVE_Y = INTAKE_HEIGHT_ABOVE;                      // 6.9
    public static double INTAKE_FAR_LOWERED_X = INTAKE_FAR_ABOVE_X;                     // 30.0
    public static double INTAKE_FAR_LOWERED_Y = INTAKE_HEIGHT_LOWERED - 2.0;            // 2.9
    public static double INTAKE_SPECIMEN_FROM_WALL_X = ARM_LENGTH;                      // 15.0
    public static double INTAKE_SPECIMEN_FROM_WALL_Y = ARM_HEIGHT - 7.0;                // 2.5
    public static double INTAKE_SPECIMEN_RAISED_X = INTAKE_SPECIMEN_FROM_WALL_X;        // 15.0
    public static double INTAKE_SPECIMEN_RAISED_Y = INTAKE_SPECIMEN_FROM_WALL_Y + 5.0;  // 7.5
    public static double NEUTRAL_X = ARM_LENGTH;                                        // 15.0
    public static double NEUTRAL_Y = ARM_HEIGHT;                                        // 9.5
    public static double START_X = ARM_LENGTH - 4.5;                                    // 10.5
    public static double START_Y = 0.5;                                                 // 0.5

    // (x, y) distances in inches from the center of rotation of the arm that the
    // scoring subsystem needs to reach to score in different places.
    public static double HIGH_BASKET_RAISED_X = ARM_LENGTH - 15.0;                      // 0.0
    public static double HIGH_BASKET_RAISED_Y = HIGH_BASKET_HEIGHT + 6.5;               // 49.5
    public static double HIGH_BASKET_RETRACTED_X = ARM_LENGTH - 15.0;                   // 0.0
    public static double HIGH_BASKET_RETRACTED_Y = ARM_HEIGHT + 0.5;                    // 10.0
    public static double HIGH_BASKET_SCORING_X = ARM_LENGTH - 18.0;                     // -3.0
    public static double HIGH_BASKET_SCORING_Y = HIGH_BASKET_RAISED_Y;                  // 49.5
    public static double HIGH_CHAMBER_SCORING_RELEASE_X = ARM_LENGTH + 2.5;             // 17.5
    public static double HIGH_CHAMBER_SCORING_RELEASE_Y = HIGH_CHAMBER_HEIGHT - 11;     // 15.0
    public static double HIGH_CHAMBER_SCORING_X = ARM_LENGTH;                           // 15.0
    public static double HIGH_CHAMBER_SCORING_Y = HIGH_CHAMBER_HEIGHT - 1;              // 25.0
    public static double LOW_ASCENT_BAR_X = ARM_LENGTH + 5.0;                           // 20.0
    public static double LOW_ASCENT_BAR_Y = LOW_ASCENT_BAR_HEIGHT + 6;                  // 26.0
    public static double LOW_BASKET_RETRACTED_X = HIGH_BASKET_RETRACTED_X;              // 0.0
    public static double LOW_BASKET_RETRACTED_Y = HIGH_BASKET_RETRACTED_Y;              // 10.0
    public static double LOW_BASKET_SCORING_X = ARM_LENGTH - 20.0;                      // -5.0
    public static double LOW_BASKET_SCORING_Y = LOW_BASKET_HEIGHT + 8;                  // 33.75

    // Time to raise or lower arm when scoring on a chamber
    public static long ARM_LOWER_TIME = 500; // milliseconds

    // Other scoring constants
    public static double ARM_HEIGHT_ADJUSTMENT_AMOUNT = 3.0; // inches

    private final Telemetry telemetry;
    private final Robot robot;
    private double targetX = START_X;
    private double targetY = START_Y;

    /**
     * Constructor
     */
    public IntakeAndScoringSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.robot = Robot.getInstance();
    }

    /**
     * Called by FTC opmode loop to take action if needed.
     */
    @Override
    public void execute() {
        // Update each components of the intake/scoring subsystem.
        robot.arm.execute();
        robot.linearSlide.execute();
        robot.wrist.execute();
        robot.claw.execute();
    }

    /**
     * Returns indication as to whether the intake and scoring subsystem has reached the target
     * position.
     *
     * @return <code>true</code> if the intake and scoring subsystem is at the target position;
     *         <code>false</code> otherwise
     */
    public boolean isAtTarget() {
        return robot.arm.isAtTarget()
                && robot.linearSlide.isAtTarget()
                && robot.wrist.isAtTarget()
                && robot.claw.isAtTarget();
    }

    /**
     * Moves the arm and slide to the position (x,y) on the plane
     * of the robot arm.  The point (0,0) on this plane is the center
     * of the shoulder around which the arm rotates.  The tip of the
     * claw should end up at the point (x,y).
     *
     * @param x The horizontal distance of the end of the claw from (0,0) along
     *          the x-axis in inches.
     * @param y The vertical distance of the end of the claw from (0,0) along
     *          the y-axis in inches.
     */
    public void moveToPosition(double x, double y) {
        x = Math.min(x, MAX_EXTENDED_ROBOT_LENGTH);

        // If the target X and/or target Y have changed, then recalculate the arm angle and
        // linear slide length
        if (targetX != x || targetY != y) {
            targetX = x;
            targetY = y;

            // Set the arm angle and slide length to reach the target X and Y coordinates
            setArmAngle();
            setSlideLength();
        }
    }

    /**
     * Calculate and set the target arm angle, accounting for the fixed height of the post on which
     * the arm is attached
     */
    private void setArmAngle() {
        double adjustedY = targetY - ARM_HEIGHT;
        double armAngle = getAngle(targetX, adjustedY);
        telemetry.addData("[IAS] arm angle", armAngle);
        robot.arm.setTargetAngle(armAngle);
    }

    /**
     * Set the linear slide length, accounting for the fixed length of the arm when the
     * linear slide is fully retracted
     */
    private void setSlideLength() {
        double adjustedY = targetY - ARM_HEIGHT;
        double slideLength = Math.hypot(targetX, adjustedY) - ARM_LENGTH;
        telemetry.addData("[IAS] slide length", slideLength);
        robot.linearSlide.setLength(slideLength);
    }

    /**
     * Moves the subsystem to the starting position, with the claw closed.
     */
    public void moveToStartPosition() {
        Robot robot = Robot.getInstance();
        moveToPosition(START_X, START_Y);
        robot.wrist.setToStartPosition();
        robot.claw.close();
        telemetry.addData("[IAS] position", "start");
    }

    /**
     * Moves the subsystem to a position where it may acquire a sample or a specimen that is
     * relative close to the front of the robot. This will lower and extend the arm so the claw
     * is above the specimen.
     */
    public void moveToIntakeCloseAboveSamplePosition() {
        moveToPosition(INTAKE_CLOSE_ABOVE_X, INTAKE_CLOSE_ABOVE_Y);
        robot.wrist.setToIntakeSample();
        robot.claw.open();
        telemetry.addData("[IAS] position", "intake close above");
    }

    /**
     * Lowers the arm from a close intake position to where the claw is in a position to pick up
     * a sample.
     */
    public void moveToIntakeCloseLoweredPosition() {
        moveToPosition(INTAKE_CLOSE_LOWERED_X, INTAKE_CLOSE_LOWERED_Y);
        robot.wrist.setToIntakeSample();
        telemetry.addData("[IAS] position", "intake close lowered");
    }

    /**
     * Moves the subsystem to a position where it may drop off a sample in the observation zone.
     */
    public void moveToDropOffSamplePosition() {
        moveToPosition(DEPOSIT_SAMPLE_X, DEPOSIT_SAMPLE_Y);
        robot.wrist.setToIntakeSample();
        telemetry.addData("[IAS] position", "intake drop-off above");
    }

    /**
     * Moves the subsystem to a position where it may acquire a sample or a specimen that is
     * relative far from the front of the robot. This will lower and extend the arm so the claw
     * may be used to pickup a sample or specimen.
     */
    public void moveToIntakeFarAboveSamplePosition() {
        moveToPosition(INTAKE_FAR_ABOVE_X, INTAKE_FAR_ABOVE_Y);
        robot.wrist.setToIntakeSample();
        robot.claw.open();
        telemetry.addData("[IAS] position", "intake far above");
    }

    /**
     * Lowers the arm from a far intake position to where the claw is in a position to pick up a
     * sample.
     */
    public void moveToIntakeFarLoweredPosition() {
        moveToPosition(INTAKE_FAR_LOWERED_X, INTAKE_FAR_LOWERED_Y);
        robot.wrist.setToIntakeSample();
        telemetry.addData("[IAS] position", "intake far lowered");
    }

    /**
     * Moves the arm to a position where it may pick up a specimen hung on the field wall.
     */
    public void moveToIntakeSpecimenOffWallPosition() {
        moveToPosition(INTAKE_SPECIMEN_FROM_WALL_X, INTAKE_SPECIMEN_FROM_WALL_Y);
        robot.wrist.setToIntakeSpecimen();
        robot.claw.open();
        telemetry.addData("[IAS] position", "intake specimen");
    }

    /**
     * Raises the arm after acquiring a specimen from the wall so that the sample is no longer
     * "clipped" to the wall.
     */
    public void moveToIntakeSpecimenRaisedPosition() {
        moveToPosition(INTAKE_SPECIMEN_RAISED_X, INTAKE_SPECIMEN_RAISED_Y);
        robot.wrist.setToIntakeSpecimen();
        telemetry.addData("[IAS] position", "intake specimen raised");
    }

    /**
     * Moves the arm to an intermediate "neutral" position that can be used as an intermediate position
     * when transitioning between various other positions.
     */
    public void moveToNeutralPosition() {
        moveToPosition(NEUTRAL_X, NEUTRAL_Y);
        robot.wrist.setToIntakeSpecimen();
        telemetry.addData("[IAS] position", "neutral");
    }

    /**
     * Moves the arm, slide, wrist, and claw to prepare the robot to score on the high basket.
     */
    public void moveToBasketHighScoringPosition() {
        moveToPosition(HIGH_BASKET_SCORING_X, HIGH_BASKET_SCORING_Y);
        robot.wrist.setToScoreBasket();
        telemetry.addData("[IAS] position", "high basket score");
    }

    /**
     * Moves the arm, slide, wrist, and claw to an extended position that is straight up and down
     * at a height that can score on the high basket. This is done prior to tiling the arm back
     * so that it can score in the basket.
     */
    public void moveToBasketHighRaisedPosition() {
        moveToPosition(HIGH_BASKET_RAISED_X, HIGH_BASKET_RAISED_Y);
        robot.wrist.setToIntakeSpecimen();
        telemetry.addData("[IAS] position", "high basket raised");
    }

    /**
     * Moves the arm, slide, wrist, and claw to a retracted position where the arm is straight up
     * and down.
     */
    public void moveToBasketHighRetractedPosition() {
        moveToPosition(HIGH_BASKET_RETRACTED_X, HIGH_BASKET_RETRACTED_Y);
        robot.wrist.setToIntakeSpecimen();
        telemetry.addData("[IAS] position", "high basket retracted");
    }

    /**
     * Moves the arm, slide, wrist, and claw to prepare the robot to score on the low basket.
     */
    public void moveToBasketLowScoringPosition() {
        moveToPosition(LOW_BASKET_SCORING_X, LOW_BASKET_SCORING_Y);
        robot.wrist.setToScoreBasket();
        telemetry.addData("[IAS] position", "low basket score");
    }

    /**
     * Moves the arm to a position where it is retracted from the low basket.
     */
    public void moveToBasketLowRetractedPosition() {
        moveToPosition(LOW_BASKET_RETRACTED_X, LOW_BASKET_RETRACTED_Y);
        robot.wrist.setToScoreBasket();
        telemetry.addData("[IAS] position", "low basket retracted");
    }

    /**
     * Moves the arm, slide, wrist, and claw to prepare the robot to score a specimen on the
     * high chamber bar.
     */
    public void moveToChamberHighScoringPosition() {
        moveToPosition(HIGH_CHAMBER_SCORING_X, HIGH_CHAMBER_SCORING_Y);
        robot.wrist.setToScoreChamber();
        telemetry.addData("[IAS] position", "high chamber score");
    }

    /**
     * Moves the arm down so as to "clip" the specimen on the high chamber bar.
     */
    public void moveToChamberHighLoweredPosition() {
        moveToPosition(HIGH_CHAMBER_SCORING_RELEASE_X, HIGH_CHAMBER_SCORING_RELEASE_Y);
        robot.wrist.setToScoreChamber();
        telemetry.addData("[IAS] position", "high chamber release");
    }

    /**
     * Moves the arm and slide to a position where it can touch the lower ascent bar.
     */
    public void moveToAscentLevelOne() {
        moveToPosition(LOW_ASCENT_BAR_X, LOW_ASCENT_BAR_Y);
    }

    /**
     * Moves the arm, slide, wrist, and claw to downward.
     */
    public void lowerArm() {
        double newY = targetY - ARM_HEIGHT_ADJUSTMENT_AMOUNT;
        newY = Math.max(newY, 0.0);
        moveToPosition(targetX, newY);
        telemetry.addData("[IAS] position", "lower arm");
    }

    /**
     * Moves the arm, slide, wrist, and claw to upward.
     */
    public void raiseArm() {
        double newY = targetY + ARM_HEIGHT_ADJUSTMENT_AMOUNT;
        newY = Math.max(newY, 0.0);
        moveToPosition(targetX, newY);
        telemetry.addData("[IAS] position", "raise arm");
    }

    /**
     * Uses the claw to grasp a scoring element (sample or specimen). The arm is not moved when
     * acquiring the scoring element; the <code>moveToIntakePosition</code> method is used for
     * that purpose.
     */
    public void closeClaw() {
        robot.claw.close();
        telemetry.addData("[IAS] claw", "close");
    }

    /**
     * Releases a scoring element (sample or specimen). This will open the claw so that the
     * scoring element is released by the subsystem. The claw will be opened, but the arm
     * is not moved; the <code>moveToIntakePosition</code> method is used for moving the arm.
     */
    public void openClaw() {
        robot.claw.open();
        telemetry.addData("[IAS] claw", "open");
    }

    /**
     * Returns a string representation of the intake and scoring subsystem.
     *
     * @return a string representation of the intake and scoring subsystem
     */
    @Override
    @NonNull
    public String toString() {
        return "IAS{" +
                robot.arm +
                ", " + robot.linearSlide +
                ", " + robot.wrist +
                ", " + robot.claw +
                "}";
    }

    /**
     * Gets an action to open the claw. This action waits for the claw to be successfully opened.
     *
     * @return an action to open the claw
     */
    public ActionEx actionOpenClaw() {
        return robot.claw.actionOpenClaw();
    }

    /**
     * Gets an action to open the claw. This action waits for the claw to be successfully opened.
     *
     * @return an action to open the claw
     */
    public ActionEx actionOpenClawWithWait() {
        return robot.claw.actionOpenClawWithWait();
    }

    /**
     * Gets an action to close the claw. This action does not wait for the claw to be successfully
     * closed.
     *
     * @return an action to close the claw
     */
    public ActionEx actionCloseClaw() {
        return robot.claw.actionCloseClaw();
    }

    /**
     * Gets an action to close the claw. This action waits for the claw to be successfully closed.
     *
     * @return an action to close the claw
     */
    public ActionEx actionCloseClawWithWait() {
        return robot.claw.actionCloseClawWithWait();
    }

    /**
     * Gets an action to touch the low ascent bar. The robot must be in the correct position prior
     * to calling this method.
     *
     * @return an action to touch the low ascent bar
     */
    public ActionEx actionTouchAscentBarLow() {
        return new MoveToAscentLevelOne(this);
    }

    /**
     * Gets an action to move the scoring subsystem to the start position.
     *
     * @return an action to move the scoring subsystem to the start position.
     */
    public ActionEx actionRetractLinearSlide() {
        return new RetractLinearSlide(this);
    }

    /**
     * Gets an action to move the scoring subsystem to the start position.
     *
     * @return an action to move the scoring subsystem to the start position.
     */
    public ActionEx actionMoveToStartPosition() {
        return new MoveToStartPosition(this);
    }

    /**
     * Gets an action to move the intake and scoring system so it is prepared to score on the
     * high chamber.
     *
     * @return an action to move the intake and scoring system so it is prepared to score on the
     *         high chamber
     */
    public ActionEx actionMoveToScoreSpecimenHighChamber() {
        return new MoveToChamberHighScoringPosition(this);
    }

    /**
     * Gets an action to score a sample in the high chamber.
     *
     * @return an action to score a sample in the high chamber
     */
    public ActionEx actionScoreSpecimenHighChamber() {
        return new SequentialAction(
                new MoveToChamberHighLoweredPosition(this),
                actionOpenClawWithWait()
        );
    }

    /**
     * Gets an action to score a sample in the high basket.
     *
     * @return an action to score a sample in the high basket
     */
    public ActionEx actionScoreSampleHighBasket() {
        return new SequentialAction(
                new MoveToBasketHighRetractedPosition(this),
                new MoveToBasketHighRaisedPosition(this),
                new MoveToBasketHighScoringPosition(this),
                actionOpenClawWithWait(),
                new MoveToBasketHighRaisedPosition(this),
                new MoveToBasketHighRetractedPosition(this),
                new MoveToNeutralPosition(this)
        );
    }

    /**
     * Gets an action to pick up a sample from a spike mark.
     *
     * @return an action to pick up a sample from a spike mark
     */
    public ActionEx actionIntakeSampleFromSpikeMark() {
        return new SequentialAction(
                new MoveToIntakeCloseAboveSamplePosition(this),
                new MoveToIntakeCloseLoweredPosition(this),
                actionCloseClawWithWait(),
                new MoveToNeutralPosition(this)
        );
    }

    /**
     * Gets an action to ove the intake and scoring system so that it can pick up a specimen off
     * the wall.
     *
     * @return an action to ove the intake and scoring system so that it can pick up a specimen off
     *         the wall
     */
    public ActionEx actionMoveToIntakeSpecimenOffWallPosition() {
        return new SequentialAction(
                new MoveToIntakeSpecimenOffWallPosition(this)
        );
    }

    /**
     * Gets an action to pick up a specimen off the wall. The intake and scoring subsystem must be
     * at the correct position for pickup, which can be accomplished by calling
     * {@link #actionMoveToIntakeSpecimenOffWallPosition()} first.
     *
     * @return an action to pick up a specimen off the wall
     */
    public ActionEx actionIntakeSpecimenFromWall() {
        return new SequentialAction(
                actionCloseClawWithWait(),
                new MoveToIntakeSpecimenRaisedPosition(this)
        );
    }

    /**
     * Gets an action to drop off a sample. This is typically used to drop off a sample in the
     * observation zone.
     *
     * @return an action to drop off a sample
     */
    public ActionEx actionDropOffSample() {
        return new SequentialAction(
                new MoveToDropOffSamplePosition(this),
                actionOpenClawWithWait(),
                new MoveToNeutralPosition(this)
        );
    }

    /**
     * Gets an action to move the arm and linear slide so the intake and scoring subsystem are at
     * the desired position.
     *
     * @param x the distance the arm needs to reach along the <code>X</code> axis
     * @param y the height the arm needs to reach along the <code>Y</code> axis
     * @return an action to move the arm and linear slide to the desired position
     */
    public ActionEx actionMoveTo(double x, double y) {
        return new MoveTo(this, x, y);
    }

    /**
     * Base class for actions that move the intake and scoring subsystem to pre-defined positions.
     */
    public static abstract class MoveToActionBase extends ActionExBase {
        protected final IntakeAndScoringSubsystem intakeAndScoringSubsystem;
        protected boolean initialized = false;
        protected ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        public MoveToActionBase(IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
            this.intakeAndScoringSubsystem = intakeAndScoringSubsystem;
        }

        /**
         * Moves the intake and scoring subsystem to the target position.
         *
         * @param telemetryPacket the telemetry used to output data to the user.
         * @return <code>true</code> if the intake and scoring subsystem are still moving to the
         *         target position; <code>false</code> if it is at the target position.
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                timer.reset();
                initialize();
                initialized = true;
            }
            boolean atTarget = isAtTarget();
            if (atTarget) {
                Robot.getInstance().telemetry.addData("[" + getClass().getSimpleName() + "] run time", FTCMath.round(timer.time(), 2));
            }
            return !atTarget;
        }

        /**
         * The intake and control system has reached it's target.
         *
         * @return <code>true</code> if the intake and scoring subsystem has reached it's target;
         *         <code>false</code> if it is still moving to it's target
         */
        public boolean isAtTarget() {
            return intakeAndScoringSubsystem.isAtTarget();
        }

        /**
         * Sets the position to which to move the intake and scoring subsystem.
         */
        public abstract void initialize();
    }

    /**
     * An action to move the intake and scoring subsystem's arm and linear slide to the desired
     * position.
     */
    public static class MoveTo extends MoveToActionBase {
        private final double x;
        private final double y;

        /**
         * Instantiates an action to move the intake and scoring subsystem to the desired position.
         *
         * @param intakeAndScoringSubsystem the intake and scoring subsystem
         * @param x                         the distance the arm needs to reach along the <code>X</code> axis
         * @param y                         the height the arm needs to reach along the <code>Y</code> axis
         */
        public MoveTo(IntakeAndScoringSubsystem intakeAndScoringSubsystem, double x, double y) {
            super(intakeAndScoringSubsystem);
            this.x = x;
            this.y = y;
        }

        /**
         * Sets the position to which to move the intake and scoring subsystem.
         */
        public void initialize() {
            intakeAndScoringSubsystem.moveToPosition(x, y);
        }
    }

    /**
     * An action to move the intake and scoring subsystem's arm to the start position.
     */
    public static class RetractLinearSlide extends MoveToActionBase {
        /**
         * Instantiates an action to move the intake and scoring subsystem's arm to the start position.
         *
         * @param intakeAndScoringSubsystem the intake and scoring subsystem
         */
        public RetractLinearSlide(IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
            super(intakeAndScoringSubsystem);
        }

        @Override
        public void initialize() {
            intakeAndScoringSubsystem.targetX = IntakeAndScoringSubsystem.ARM_LENGTH;
            intakeAndScoringSubsystem.setSlideLength();
        }
    }

    /**
     * An action to move the intake and scoring subsystem's arm to the start position.
     */
    public static class MoveToStartPosition extends MoveToActionBase {
        /**
         * Instantiates an action to move the intake and scoring subsystem's arm to the start position.
         *
         * @param intakeAndScoringSubsystem the intake and scoring subsystem
         */
        public MoveToStartPosition(IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
            super(intakeAndScoringSubsystem);
        }

        @Override
        public void initialize() {
            intakeAndScoringSubsystem.moveToStartPosition();
        }
    }

    /**
     * An action to move the intake and scoring subsystem's arm to the neutral position.
     */
    public static class MoveToNeutralPosition extends MoveToActionBase {
        /**
         * Instantiates an action to move the intake and scoring subsystem's arm to the neutral position.
         *
         * @param intakeAndScoringSubsystem the intake and scoring subsystem
         */
        public MoveToNeutralPosition(IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
            super(intakeAndScoringSubsystem);
        }

        @Override
        public void initialize() {
            intakeAndScoringSubsystem.moveToNeutralPosition();
        }
    }

    /**
     * An action to move the intake and scoring subsystem's arm to intake a sample relatively
     * close to the robot.
     */
    public static class MoveToIntakeCloseAboveSamplePosition extends MoveToActionBase {
        private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        /**
         * Instantiates an action to move the intake and scoring subsystem's arm to intake a sample
         * relatively close to the robot.
         *
         * @param intakeAndScoringSubsystem the intake and scoring subsystem
         */
        public MoveToIntakeCloseAboveSamplePosition(IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
            super(intakeAndScoringSubsystem);
        }

        @Override
        public void initialize() {
            intakeAndScoringSubsystem.moveToIntakeCloseAboveSamplePosition();
            timer.reset();
        }

        @Override
        public boolean isAtTarget() {
            double elapsedTime = timer.time();
            return intakeAndScoringSubsystem.isAtTarget() && elapsedTime >= INTAKE_WAIT_TIME;
        }
    }

    /**
     * An action to lower the intake and scoring subsystem's arm to intake a sample relatively
     * close to the robot.
     */
    public static class MoveToIntakeCloseLoweredPosition extends MoveToActionBase {
        private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        /**
         * Instantiates an action to lower the intake and scoring subsystem's arm to intake a sample
         * relatively close to the robot.
         *
         * @param intakeAndScoringSubsystem the intake and scoring subsystem
         */
        public MoveToIntakeCloseLoweredPosition(IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
            super(intakeAndScoringSubsystem);
        }

        @Override
        public void initialize() {
            intakeAndScoringSubsystem.moveToIntakeCloseLoweredPosition();
            timer.reset();
        }

        @Override
        public boolean isAtTarget() {
            double elapsedTime = timer.time();
            return intakeAndScoringSubsystem.isAtTarget() && elapsedTime >= INTAKE_WAIT_TIME;
        }
    }

    /**
     * An action to move the intake and scoring subsystem's arm to intake a sample relatively far
     * from the robot.
     */
    public static class MoveToIntakeFarAboveSamplePosition extends MoveToActionBase {
        /**
         * Instantiates an action to move the intake and scoring subsystem's arm to intake a sample
         * relatively far from the robot.
         *
         * @param intakeAndScoringSubsystem the intake and scoring subsystem
         */
        public MoveToIntakeFarAboveSamplePosition(IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
            super(intakeAndScoringSubsystem);
        }

        @Override
        public void initialize() {
            intakeAndScoringSubsystem.moveToIntakeFarAboveSamplePosition();
        }
    }

    /**
     * An action to lower the intake and scoring subsystem's arm to intake a sample relatively
     * far from the robot.
     */
    public static class MoveToIntakeFarLoweredPosition extends MoveToActionBase {
        /**
         * Instantiates an action to lower the intake and scoring subsystem's arm to intake a sample
         * relatively far from the robot.
         *
         * @param intakeAndScoringSubsystem the intake and scoring subsystem
         */
        public MoveToIntakeFarLoweredPosition(IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
            super(intakeAndScoringSubsystem);
        }

        @Override
        public void initialize() {
            intakeAndScoringSubsystem.moveToIntakeFarLoweredPosition();
        }
    }

    /**
     * An action to move the intake and scoring subsystem's arm to intake a specimen from the wall.
     */
    public static class MoveToIntakeSpecimenOffWallPosition extends MoveToActionBase {
        /**
         * Instantiates an action to move the intake and scoring subsystem's arm to intake a specimen
         * from the wall.
         *
         * @param intakeAndScoringSubsystem the intake and scoring subsystem
         */
        public MoveToIntakeSpecimenOffWallPosition(IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
            super(intakeAndScoringSubsystem);
        }

        @Override
        public void initialize() {
            intakeAndScoringSubsystem.moveToIntakeSpecimenOffWallPosition();
        }
    }

    /**
     * An action to raise the intake and scoring subsystem's arm after picking up a specimen from
     * the wall.
     */
    public static class MoveToIntakeSpecimenRaisedPosition extends MoveToActionBase {
        /**
         * Instantiates an action to raise the intake and scoring subsystem's arm after picking up a
         * specimen from the wall.
         *
         * @param intakeAndScoringSubsystem the intake and scoring subsystem
         */
        public MoveToIntakeSpecimenRaisedPosition(IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
            super(intakeAndScoringSubsystem);
        }

        @Override
        public void initialize() {
            intakeAndScoringSubsystem.moveToIntakeSpecimenRaisedPosition();
        }
    }

    /**
     * An action to move the intake and scoring subsystem's arm to drop off a sample.
     */
    public static class MoveToDropOffSamplePosition extends MoveToActionBase {
        /**
         * Instantiates an action to move the intake and scoring subsystem's arm to drop off a sample.
         *
         * @param intakeAndScoringSubsystem the intake and scoring subsystem
         */
        public MoveToDropOffSamplePosition(IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
            super(intakeAndScoringSubsystem);
        }

        @Override
        public void initialize() {
            intakeAndScoringSubsystem.moveToDropOffSamplePosition();
        }
    }

    /**
     * An action to raise the intake and scoring subsystem's arm to prepare to score in the high basket.
     */
    public static class MoveToBasketHighRaisedPosition extends MoveToActionBase {
        /**
         * Instantiates an action to raise the intake and scoring subsystem's arm to prepare to score
         * in the high basket.
         *
         * @param intakeAndScoringSubsystem the intake and scoring subsystem
         */
        public MoveToBasketHighRaisedPosition(IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
            super(intakeAndScoringSubsystem);
        }

        @Override
        public void initialize() {
            intakeAndScoringSubsystem.moveToBasketHighRaisedPosition();
        }
    }

    /**
     * An action to move the intake and scoring subsystem's arm to score in the high basket.
     */
    public static class MoveToBasketHighScoringPosition extends MoveToActionBase {
        /**
         * Instantiates an action to move the intake and scoring subsystem's arm to score in the
         * high basket.
         *
         * @param intakeAndScoringSubsystem the intake and scoring subsystem
         */
        public MoveToBasketHighScoringPosition(IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
            super(intakeAndScoringSubsystem);
        }

        @Override
        public void initialize() {
            intakeAndScoringSubsystem.moveToBasketHighScoringPosition();
        }
    }

    /**
     * An action to retract the intake and scoring subsystem's arm to after scoring in the high basket.
     */
    public static class MoveToBasketHighRetractedPosition extends MoveToActionBase {
        /**
         * Instantiates an action to retract the intake and scoring subsystem's arm to after scoring
         * in the high basket.
         *
         * @param intakeAndScoringSubsystem the intake and scoring subsystem
         */
        public MoveToBasketHighRetractedPosition(IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
            super(intakeAndScoringSubsystem);
        }

        @Override
        public void initialize() {
            intakeAndScoringSubsystem.moveToBasketHighRetractedPosition();
        }
    }

    /**
     * An action to raise the intake and scoring subsystem's arm to prepare to score in the low basket.
     */
    public static class MoveToBasketLowRaisedPosition extends MoveToActionBase {
        /**
         * Instantiates an action to raise the intake and scoring subsystem's arm to prepare to score
         * in the low basket.
         *
         * @param intakeAndScoringSubsystem the intake and scoring subsystem
         */
        public MoveToBasketLowRaisedPosition(IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
            super(intakeAndScoringSubsystem);
        }

        @Override
        public void initialize() {
            intakeAndScoringSubsystem.moveToBasketHighRaisedPosition();
        }
    }

    /**
     * An action to move the intake and scoring subsystem's arm to score in the low basket.
     */
    public static class MoveToBasketLowScoringPosition extends MoveToActionBase {
        /**
         * Instantiates an action to move the intake and scoring subsystem's arm to score in the
         * low basket.
         *
         * @param intakeAndScoringSubsystem the intake and scoring subsystem
         */
        public MoveToBasketLowScoringPosition(IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
            super(intakeAndScoringSubsystem);
        }

        @Override
        public void initialize() {
            intakeAndScoringSubsystem.moveToBasketLowScoringPosition();
        }
    }

    /**
     * An action to retract the intake and scoring subsystem's arm to after scoring in the low basket.
     */
    public static class MoveToBasketLowRetractedPosition extends MoveToActionBase {
        /**
         * Instantiates an action to retract the intake and scoring subsystem's arm to after scoring
         * in the low basket.
         *
         * @param intakeAndScoringSubsystem the intake and scoring subsystem
         */
        public MoveToBasketLowRetractedPosition(IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
            super(intakeAndScoringSubsystem);
        }

        @Override
        public void initialize() {
            intakeAndScoringSubsystem.moveToBasketLowRetractedPosition();
        }
    }

    /**
     * An action to raise the intake and scoring subsystem's arm to prepare to score on the high chamber.
     */
    public static class MoveToChamberHighScoringPosition extends MoveToActionBase {
        /**
         * Instantiates an action to raise the intake and scoring subsystem's arm to prepare to score
         * on the high chamber.
         *
         * @param intakeAndScoringSubsystem the intake and scoring subsystem
         */
        public MoveToChamberHighScoringPosition(IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
            super(intakeAndScoringSubsystem);
        }

        @Override
        public void initialize() {
            intakeAndScoringSubsystem.moveToChamberHighScoringPosition();
        }
    }

    /**
     * An action to lower the intake and scoring subsystem's arm to clip a specimen on the high chamber.
     */
    @Config
    public static class MoveToChamberHighLoweredPosition extends MoveToActionBase {
        private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        /**
         * Instantiates an action to lower the intake and scoring subsystem's arm to clip a specimen
         * on the high chamber.
         *
         * @param intakeAndScoringSubsystem the intake and scoring subsystem
         */
        public MoveToChamberHighLoweredPosition(IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
            super(intakeAndScoringSubsystem);
        }

        @Override
        public void initialize() {
            intakeAndScoringSubsystem.moveToChamberHighLoweredPosition();
            timer.reset();
        }

        @Override
        public boolean isAtTarget() {
            double elapsedTime = timer.time();
            boolean atTarget = elapsedTime >= ARM_LOWER_TIME;
            FtcDashboard.getInstance().getTelemetry().addData("[Lower Arm] atTarget", atTarget);
            return atTarget;
        }
    }

    /**
     * An action to raise the intake and scoring subsystem's arm to touch the low ascent bar, achieving
     * a Level 1 ascent.
     */
    public static class MoveToAscentLevelOne extends MoveToActionBase {
        /**
         * Instantiates an action to raise the intake and scoring subsystem's arm to touch the
         * low ascent bar, achieving a Level 1 ascent.
         *
         * @param intakeAndScoringSubsystem the intake and scoring subsystem
         */
        public MoveToAscentLevelOne(IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
            super(intakeAndScoringSubsystem);
        }

        @Override
        public void initialize() {
            intakeAndScoringSubsystem.moveToAscentLevelOne();
        }
    }
}
