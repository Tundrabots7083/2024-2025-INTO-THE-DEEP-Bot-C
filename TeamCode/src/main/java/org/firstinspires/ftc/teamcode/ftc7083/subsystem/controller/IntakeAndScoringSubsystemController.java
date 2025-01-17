package org.firstinspires.ftc.teamcode.ftc7083.subsystem.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;

/**
 * Manages the intake and scoring subsystem within a TeleOp OpMode. The following controls are
 * used to manage the subsystem:
 * <ul>
 *           <em>gamepad2.dpad_up</em>: a toggle that moves the scoring subsystem to the high chamber
 *  *          position when first pressed and retracts the linear slide if pressed a second time.
 *     </li>
 *     <li>
 *         <em>gamepad2.cross</em>: a toggle that moves the scoring subsystem to the low basket
 *          position when first pressed and retracts the linear slide if pressed a second time.
 *     </li>
 *     <li>
 *         <em>gamepad2.triangle</em>: a toggle that moves the scoring subsystem to the high basket
 *  *          position when first pressed and retracts the linear slide if pressed a second time.
 *     </li>
 *     <li>
 *         <em>gamepad2.square</em>: a toggle that moves the scoring subsystem into the submersible
 *         at a position relatively close to the robot when first pressed and retracts the linear
 *         slide if pressed a second time.
 *     </li>
 *     <li>
 *         <em>gamepad2.circle</em>: a toggle that moves the scoring subsystem into the submersible
 *  *         at a position relatively far from the robot when first pressed and retracts the linear
 *  *         slide if pressed a second time.
 *     </li>
 *     <li>
 *         <em>gamepad2.share</em>: move the scoring subsystem to a neutral position, where the arm is
 *         extended horizontal to the ground and the linear slide is fully retracted.
 *     </li>
 *     <li>
 *         <em>gamepad2.option</em>: move the scoring subsystem to a starting position, where the arm is
 *         all the way down and the linear slide is fully retracted.
 *     </li>
 *     <li>
 *         <em>gamepad2.dpad_left</em>: lowers the arm on the scoring subsystem.
 *     </li>
 *     <li>
 *         <em>gamepad2.dpad_right</em>: raises the arm on the scoring subsystem.
 *     </li>
 *     <li>
 *         <em>gamepad2.left_bumper</em>: open or close the claw.
 *     </li>
 *     <li>
 *         <em>gamepad2.right_bumper</em>: move the scoring subsystem into a position where the
 *         linear slide can touch the low chamber bar, achieving an ascent level 1.
 *     </li>
 * </ul>
 */
@Config
public class IntakeAndScoringSubsystemController implements SubsystemController {
    private final IntakeAndScoringSubsystem intakeAndScoringSubsystem;
    private final Telemetry telemetry;

    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();

    private State state = State.NEUTRAL_POSITION;
    private boolean clawOpen = false;

    /**
     * Instantiate a scoring subsystem controller, which uses gamepad controls to control the
     * scoring subsystem.
     *
     * @param intakeAndScoringSubsystem the scoring subsystem being controlled
     * @param telemetry                 the telemetry used to provide user output on the driver station and FTC dashboard
     */
    public IntakeAndScoringSubsystemController(IntakeAndScoringSubsystem intakeAndScoringSubsystem, Telemetry telemetry) {
        this.intakeAndScoringSubsystem = intakeAndScoringSubsystem;
        this.telemetry = telemetry;
    }

    /**
     * Updates the robot drive speed based on the left and right joysticks on gamepad1. In addition,
     * the left bumper on gamepad1 can be used to toggle the gain for the X axis, Y axis and
     * turn speed.
     *
     * @param gamepad1 Gamepad1
     * @param gamepad2 Gamepad2
     */
    public void execute(Gamepad gamepad1, Gamepad gamepad2) {
        // Preset positions for the arm and linear slide
        if (gamepad2.dpad_up && !previousGamepad2.dpad_up) {
            if (state != State.HIGH_CHAMBER_SCORING_POSITION) {
                intakeAndScoringSubsystem.moveToChamberHighScoringPosition();
                state = State.HIGH_CHAMBER_SCORING_POSITION;
            } else {
                intakeAndScoringSubsystem.retractLinearSlide();
                state = State.RETRACTED_POSITION;
            }
        } else if (gamepad2.cross && !previousGamepad2.cross) {
            if (state != State.LOW_BASKET_SCORING_POSITION) {
                intakeAndScoringSubsystem.moveToBasketLowScoringPosition();
                state = State.LOW_BASKET_SCORING_POSITION;
            } else {
                intakeAndScoringSubsystem.retractLinearSlide();
                state = State.RETRACTED_POSITION;
            }
        } else if (gamepad2.triangle && !previousGamepad2.triangle) {
            if (state != State.HIGH_BASKET_SCORING_POSITION) {
                intakeAndScoringSubsystem.moveToBasketHighScoringPosition();
                state = State.HIGH_BASKET_SCORING_POSITION;
            } else {
                intakeAndScoringSubsystem.retractLinearSlide();
                state = State.RETRACTED_POSITION;
            }
        } else if (gamepad2.square && !previousGamepad2.square) {
            if (state != State.INTAKE_CLOSE_POSITION) {
                intakeAndScoringSubsystem.moveToIntakeShortPosition();
                state = State.INTAKE_CLOSE_POSITION;
            } else {
                intakeAndScoringSubsystem.retractFromSubmersible();
                state = State.RETRACTED_POSITION;
            }
        } else if (gamepad2.circle && !previousGamepad2.circle) {
            if (state != State.INTAKE_FAR_POSITION) {
                intakeAndScoringSubsystem.moveToIntakeLongPosition();
                state = State.INTAKE_FAR_POSITION;
            } else {
                intakeAndScoringSubsystem.retractFromSubmersible();
                state = State.RETRACTED_POSITION;
            }
        } else if (gamepad2.right_bumper && !previousGamepad2.right_bumper) {
            if (state != State.ASCENT_LEVEL_ONE) {
                intakeAndScoringSubsystem.moveToAscentLevelOne();
                state = State.ASCENT_LEVEL_ONE;
            } else {
                intakeAndScoringSubsystem.retractLinearSlide();
                state = State.RETRACTED_POSITION;
            }
        } else if (gamepad2.share && !previousGamepad2.share) {
            intakeAndScoringSubsystem.moveToNeutralPosition();
            state = State.NEUTRAL_POSITION;
        } else if (gamepad2.options) {
            intakeAndScoringSubsystem.moveToStartPosition();
            state = State.START_POSITION;
        } else if (gamepad2.dpad_left && !previousGamepad2.dpad_left) {
            intakeAndScoringSubsystem.raiseArm();
        } else if (gamepad2.dpad_right && !previousGamepad2.dpad_right) {
            intakeAndScoringSubsystem.lowerArm();
        }

        // Manual override controls for the arm and linear slide. The left joystick will raise and
        // lower the arm; the right joystick will extend and retract the linear slide.
       /* if (Math.abs(gamepad2.left_stick_y) > MIN_JOYSTICK_VALUE) {
            double adjustY = MANUAL_Y_ADJUSTMENT;
            if (gamepad2.left_stick_y > 0.0) {
                adjustY *= -1;
            }
            intakeAndScoringSubsystem.adjustY(adjustY);
            telemetry.addData("[IAS C] adj arm height", adjustY);
        }
        if (Math.abs(gamepad2.right_stick_y) > MIN_JOYSTICK_VALUE) {
            double adjustX = MANUAL_X_ADJUSTMENT;
            if (gamepad2.right_stick_y > 0.0) {
                adjustX *= -1;
            }
            intakeAndScoringSubsystem.adjustX(adjustX);
            telemetry.addData("[IAS C] adj slide length", adjustX);
        }*/

        // Open and close the claw; used for acquiring samples/specimens and scoring
        // or depositing them
        if (gamepad2.left_bumper && !previousGamepad2.left_bumper) {
            if (clawOpen) {
                intakeAndScoringSubsystem.closeClaw();
                clawOpen = false;
            } else {
                intakeAndScoringSubsystem.openClaw();
                clawOpen = true;
            }
        }

        // Update the scoring subsystem. This allows it to adjust the position of the managed
        // components as they move to a target position.
        intakeAndScoringSubsystem.execute();

        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }

    /**
     * State of the intake and scoring subsystem
     */
    enum State {
        START_POSITION,
        NEUTRAL_POSITION,
        RETRACTED_POSITION,
        LOW_BASKET_SCORING_POSITION,
        HIGH_BASKET_SCORING_POSITION,
        LOW_CHAMBER_SCORING_POSITION,
        HIGH_CHAMBER_SCORING_POSITION,
        INTAKE_CLOSE_POSITION,
        INTAKE_FAR_POSITION,
        ASCENT_LEVEL_ONE
    }
}
