package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionEx;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionExBase;
import org.firstinspires.ftc.teamcode.ftc7083.action.SequentialAction;

/**
 * This class uses the Arm, LinearSlide, Wrist, and Claw subsystems to pick up, control,
 * and score samples and specimens.
 */
@Config
public class IntakeAndScoringSubsystem extends SubsystemBase {

    // Robot measurements
    // Length of the arm and the wrist with zero extension in inches.
    public static double ARM_LENGTH = 15.0;
    // Height from the field to the center of rotation of the arm in inches.
    public static double ARM_HEIGHT = 9.5;
    // Distance from the front of the robot to the back of the robot in inches.
    public static double ROBOT_LENGTH = 14.25;
    // Distance the arm can reach from the center of rotation of the arm in inches.
    public static double MAX_ARM_LENGTH = 43.5;

    // Intake constants
    public static double START_X = 14;
    public static double START_Y = 0.0;
    public static double NEUTRAL_X = ARM_LENGTH;
    public static double NEUTRAL_Y = ARM_HEIGHT;
    public static double RETRACT_X = ARM_LENGTH;
    public static double RETRACT_Y = 5.0;
    public static double INTAKE_SHORT_X = 27.0;
    public static double INTAKE_SHORT_Y = 1.6;
    public static double INTAKE_LONG_X = 36.0;
    public static double INTAKE_LONG_Y = 1.6;
    public static double DEPOSIT_SAMPLE_X = 27.0;
    public static double DEPOSIT_SAMPLE_Y = 1.6;

    // Heights of scoring places for game are in inches
    public static double HIGH_CHAMBER_HEIGHT = 26.0;
    public static double LOW_CHAMBER_HEIGHT = 13.0;
    public static double HIGH_BASKET_HEIGHT = 48.6;
    public static double LOW_BASKET_HEIGHT = 25.75;
    public static double LOW_ASCENT_BAR_HEIGHT = 20.0;
    public static double HIGH_ASCENT_BAR_HEIGHT = 36.0;

    // Maximum horizontal length of robot when extended
    public static double MAX_EXTENDED_ROBOT_LENGTH = 40.0;

    // (x, y) distances in inches from the center of rotation of
    // the arm that the scoring subsystem needs to reach to score in
    // different places.
    public static double HIGH_CHAMBER_SCORING_X = ARM_LENGTH;
    public static double HIGH_CHAMBER_SCORING_Y = HIGH_CHAMBER_HEIGHT + 2.5;
    public static double HIGH_CHAMBER_SCORING_RELEASE_Y = HIGH_CHAMBER_SCORING_Y - 10;
    public static double LOW_CHAMBER_SCORING_X = ARM_LENGTH;
    public static double LOW_CHAMBER_SCORING_Y = LOW_CHAMBER_HEIGHT + 3.5;
    public static double HIGH_BASKET_SCORING_X = ARM_LENGTH;
    public static double HIGH_BASKET_SCORING_Y = HIGH_BASKET_HEIGHT + 5.5;
    public static double LOW_BASKET_SCORING_X = ARM_LENGTH;
    public static double LOW_BASKET_SCORING_Y = LOW_BASKET_HEIGHT + 5.5;
    public static double OBSERVATION_ZONE_INTAKE_SPECIMEN_X = ARM_LENGTH + 6.0;
    public static double OBSERVATION_ZONE_INTAKE_SPECIMEN_GRAB_Y = 4.5;
    public static double OBSERVATION_ZONE_INTAKE_SPECIMEN_ACQUIRE_Y = OBSERVATION_ZONE_INTAKE_SPECIMEN_GRAB_Y + 5;
    public static double LOW_ASCENT_BAR_X = ARM_LENGTH;
    public static double LOW_ASCENT_BAR_Y = LOW_ASCENT_BAR_HEIGHT;

    // Other scoring constants
    public static double MOVE_ARM_AMOUNT = 3.0;

    private final Telemetry telemetry;
    private final Robot robot;
    private double targetX = RETRACT_X;
    private double targetY = RETRACT_Y;

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

        telemetry.addData("[IAS] target X", targetX);
        telemetry.addData("[IAS] target Y", targetY);
    }

    /**
     * Returns indication as to whether the intake and scoring subsystem has reached the target
     * position.
     *
     * @return <code>true</code> if the intake and scoring subsystem is at the target position;
     *         <code>false</code> otherwise
     */
    public boolean isAtTarget() {
        return robot.arm.isAtTarget() && robot.linearSlide.isAtTarget();
    }

    /**
     * Moves the subsystem to the starting position, with the claw closed.
     */
    public void moveToStartPosition() {
        moveToPosition(START_X, START_Y);
        telemetry.addData("[IAS] position", "start");
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
        // Don't use the wrist movement, as the arm can't reach the ground to pickup samples if the
        // wrist is articulated
        // robot.wrist.setPitch(-armAngle * 2.0);
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
     * Fully retract the linear slide while not changing the arm angle.
     */
    public void retractLinearSlide() {
        // Get angle on the triangle. This is not going to change as the arm is retracted, as the
        // new triangle will be complementary to the current triangle.
        double angle = robot.arm.getCurrentAngle();

        // Fully retract the slide by setting the hypotenuse to the arm length, which will effectively
        // set the slide length to zero. Use the hypotenuse to calculate the new X and Y coordinates
        // for the right triangle. ARM_HEIGHT is added to the Y value, as the ARM_HEIGHT is
        // subtracted from the Y value by the moveToPosition() method.
        double hypotenuse = ARM_LENGTH;
        double x = getX(angle, hypotenuse);
        double y = getY(angle, hypotenuse) + ARM_HEIGHT;

        moveToPosition(x, y);
        telemetry.addData("[IAS] position", "retract slide");
    }

    /**
     * Retracts the linear slide from the submersible, slightly raising the arm so the slide does
     * not hit the 2" lip around the submersible.
     */
    public void retractFromSubmersible() {
        moveToPosition(RETRACT_X, RETRACT_Y);
        telemetry.addData("[IAS] position", "retract submersible");
    }

    /**
     * Moves the subsystem to a position where it may acquire a sample or a specimen that is
     * relative close to the front of the robot. This will lower and extend the arm so the claw
     * may be used to pickup a sample or specimen.
     */
    public void moveToIntakeShortPosition() {
        moveToPosition(INTAKE_SHORT_X, INTAKE_SHORT_Y);
        telemetry.addData("[IAS] position", "intake short");
    }

    /**
     * Moves the subsystem to a position where it may acquire a sample or a specimen that is
     * relative far from the front of the robot. This will lower and extend the arm so the claw
     * may be used to pickup a sample or specimen.
     */
    public void moveToIntakeLongPosition() {
        moveToPosition(INTAKE_LONG_X, INTAKE_LONG_Y);
        telemetry.addData("[IAS] position", "intake long");
    }

    /**
     * Moves the arm after a sample or specimen has been picked up. This will raise and retract the
     * arm so the robot may be maneuvered into a scoring position by the IntakeAndScoringSubsystem.
     */
    public void moveToNeutralPosition() {
        moveToPosition(NEUTRAL_X, NEUTRAL_Y);
        telemetry.addData("[IAS] position", "neutral");
    }

    /**
     * Moves the arm, slide, wrist, and claw to prepare the robot
     * to score on the high basket.
     */
    public void moveToBasketHighScoringPosition() {
        moveToPosition(HIGH_BASKET_SCORING_X, HIGH_BASKET_SCORING_Y);
        telemetry.addData("[IAS] position", "high basket");
    }

    /**
     * Moves the arm, slide, wrist, and claw to prepare the robot
     * to score on the low basket.
     */
    public void moveToBasketLowScoringPosition() {
        moveToPosition(LOW_BASKET_SCORING_X, LOW_BASKET_SCORING_Y);
        telemetry.addData("[IAS] position", "low basket");
    }

    /**
     * Moves the arm, slide, wrist, and claw to prepare the robot
     * to score on the high chamber bar.
     */
    public void moveToChamberHighScoringPosition() {
        moveToPosition(HIGH_CHAMBER_SCORING_X, HIGH_CHAMBER_SCORING_Y);
        telemetry.addData("[IAS] position", "high chamber");
    }

    /**
     * Moves the arm, slide, wrist, and claw to prepare the robot
     * to score on the low chamber bar.
     */
    public void moveToChamberLowScoringPosition() {
        moveToPosition(LOW_CHAMBER_SCORING_X, LOW_CHAMBER_SCORING_Y);
        telemetry.addData("[IAS] position", "low chamber");
    }

    /**
     * Moves the arm, slide, and wrist to prepare the robot
     * to intake a sample.
     */
//    public void moveToSampleIntakePosition() {
//        double xDistance = (double) robot.limelight.getDistance(Limelight.TargetPosition.SUBMERSIBLE);
//        double sampleHeight = 1.1;
//        moveToPosition(xDistance,sampleHeight);
//    }

    /**
     * Moves the arm, slide, wrist, and claw to downward.
     */
    public void lowerArm() {
        double newY = targetY - MOVE_ARM_AMOUNT;
        newY = Math.max(newY, 0.0);
        moveToPosition(targetX, newY);
        telemetry.addData("[IAS] position", "lower arm");
    }

    /**
     * Moves the arm, slide, wrist, and claw to upward.
     */
    public void raiseArm() {
        double newY = targetY + MOVE_ARM_AMOUNT;
        newY = Math.max(newY, 0.0);
        moveToPosition(targetX, newY);
        telemetry.addData("[IAS] position", "raise arm");
    }

    /**
     * Increases the slide length by the amount provided. A positive value extends the slide;
     * a negative value retracts the arm.
     *
     * @param amount the amount by which to adjust the arm angle.
     */
    public void adjustX(double amount) {
        targetX += amount;
        setSlideLength();
    }

    /**
     * Increases the arm angle by the amount provided. A positive value moves the arm up; a negative
     * value moves the arm down.
     *
     * @param amount the amount by which to adjust the arm angle.
     */
    public void adjustY(double amount) {
        targetY += amount;
        setArmAngle();
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
     * Gets an action to score a specimen on the high chamber. The robot must be in the correct
     * scoring position prior to calling this method.
     *
     * @return an action to score a specimen on the high chamber
     */
    public ActionEx actionScoreSpecimenHighChamber() {
        return new SequentialAction(
                new MoveTo(this, HIGH_CHAMBER_SCORING_X, HIGH_CHAMBER_SCORING_Y),
                new MoveTo(this, HIGH_BASKET_SCORING_X, HIGH_CHAMBER_SCORING_RELEASE_Y),
                actionOpenClawWithWait(),
                actionRetractLinearSlide()
        );
    }

    /**
     * Gets an action to score a specimen on the high chamber. The robot must be in the correct
     * scoring position prior to calling this method.
     *
     * @return an action to score a specimen on the high chamber
     */
    public ActionEx actionScoreSpecimenHighBasket() {
        return new SequentialAction(
                new MoveTo(this, HIGH_BASKET_SCORING_X, HIGH_BASKET_SCORING_Y),
                actionOpenClawWithWait(),
                actionRetractLinearSlide()
        );
    }

    /**
     * Gets an action to intake a specimen from the ground. The robot must be in the
     * correct position prior to calling this method.
     *
     * @return an action to intake a specimen from the observation zone wall
     */
    public ActionEx actionIntakeSample() {
        return new SequentialAction(
                // TODO: this is not correct - it will need to use the LimeLight camera to orient the wrist and
                //       adjust the length of the linear slide
                new MoveTo(this, INTAKE_SHORT_X, INTAKE_SHORT_Y),
                actionCloseClawWithWait(),
                actionRetractLinearSlide()
        );
    }

    /**
     * Gets an action to intake a specimen from the ground. The robot must be in the
     * correct position prior to calling this method.
     *
     * @return an action to intake a specimen from the observation zone wall
     */
    public ActionEx actionDepositSample() {
        return new SequentialAction(
                new MoveTo(this, DEPOSIT_SAMPLE_X, DEPOSIT_SAMPLE_Y),
                actionOpenClawWithWait(),
                actionRetractLinearSlide()
        );
    }

    /**
     * Gets an action to intake a specimen from the observation zone wall. The robot must be in the
     * correct position prior to calling this method.
     *
     * @return an action to intake a specimen from the observation zone wall
     */
    public ActionEx actionIntakeSpecimen() {
        return new SequentialAction(
                // TODO: this is not correct - it will need to use the LimeLight camera to orient the wrist and
                //       adjust the length of the linear slide
                new MoveTo(this, OBSERVATION_ZONE_INTAKE_SPECIMEN_X, OBSERVATION_ZONE_INTAKE_SPECIMEN_GRAB_Y),
                actionCloseClawWithWait(),
                new MoveTo(this, OBSERVATION_ZONE_INTAKE_SPECIMEN_X, OBSERVATION_ZONE_INTAKE_SPECIMEN_ACQUIRE_Y),
                actionRetractLinearSlide()
        );
    }

    /**
     * Gets an action to touch the low ascent bar. The robot must be in the correct position prior
     * to calling this method.
     *
     * @return an action to touch the low ascent bar
     */
    public ActionEx actionTouchAscentBarLow() {
        return new MoveTo(this, LOW_ASCENT_BAR_X, LOW_ASCENT_BAR_Y);
    }

    /**
     * Gets an action to move the scoring subsystem to the start position.
     *
     * @return an action to move the scoring subsystem to the start position.
     */
    public ActionEx actionMoveToStartPosition() {
        return new MoveTo(this, START_X, START_Y);
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
     * Gets an action that retracts the linear slide while maintaining the current angle of the arm.
     * @return an action that retracts the linear slide while maintaining the current angle of the arm
     */
    public ActionEx actionRetractLinearSlide() {
        return new RetractLinearSlide(this);
    }

    /**
     * An action to move the intake and scoring subsystem's arm and linear slide to the desired
     * position.
     */
    public static class MoveTo extends ActionExBase {
        private final IntakeAndScoringSubsystem intakeAndScoringSubsystem;
        private final double x;
        private final double y;
        private boolean initialized = false;

        /**
         * Instantiates an action to move the intake and scoring subsystem to the desired position.
         *
         * @param intakeAndScoringSubsystem the intake and scoring subsystem
         * @param x                         the distance the arm needs to reach along the <code>X</code> axis
         * @param y                         the height the arm needs to reach along the <code>Y</code> axis
         */
        public MoveTo(IntakeAndScoringSubsystem intakeAndScoringSubsystem, double x, double y) {
            this.intakeAndScoringSubsystem = intakeAndScoringSubsystem;
            this.x = x;
            this.y = y;
        }

        /**
         * Moves the intake and scoring subsystem to the desired position.
         *
         * @param telemetryPacket the telemetry used to output data to the user.
         * @return <code>true</code> if the intake and scoring subsystem are still moving to the
         *         target position; <code>false</code> if it is at the target position.
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialize();
            }
            intakeAndScoringSubsystem.execute();
            return !intakeAndScoringSubsystem.isAtTarget();
        }

        /**
         * Sets the position to which to move the intake and scoring subsystem.
         */
        private void initialize() {
            intakeAndScoringSubsystem.moveToPosition(x, y);
            initialized = true;
        }
    }

    /**
     * An action that retracts the linear slide while maintaining the current angle of the arm.
     */
    public static class RetractLinearSlide extends ActionExBase {
        private final IntakeAndScoringSubsystem intakeAndScoringSubsystem;
        private boolean initialized = false;

        /**
         * Instantiates an action that retracts the linear slide while maintaining the current angle of the arm.
         *
         * @param intakeAndScoringSubsystem the intake and scoring subsystem
         */
        public RetractLinearSlide(IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
            this.intakeAndScoringSubsystem = intakeAndScoringSubsystem;
        }

        /**
         * Retracts the linear slide while maintaining the current angle of the arm.
         *
         * @param telemetryPacket the telemetry used to output data to the user.
         * @return <code>true</code> if the intake and scoring subsystem are still moving to the
         *         target position; <code>false</code> if it is at the target position.
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialize();
            }
            intakeAndScoringSubsystem.execute();
            return !intakeAndScoringSubsystem.isAtTarget();
        }

        /**
         * Sets the position to which to move the intake and scoring subsystem.
         */
        private void initialize() {
            intakeAndScoringSubsystem.retractLinearSlide();
            initialized = true;
        }
    }
}
