package org.firstinspires.ftc.teamcode.ftc7083.subsystem.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;

/**
 * Manages the intake and scoring subsystem within a TeleOp OpMode. The following controls are
 * used to manage the subsystem:
 * <ul>
 *     <li>
 *          <em>gamepad2.dpad_up</em>: a toggle that moves the scoring subsystem to the high chamber
 *          position when first pressed and retracts the linear slide if pressed a second time.
 *     </li>
 *     <li>
 *         <em>gamepad2.cross</em>: a toggle that moves the scoring subsystem to the low basket
 *          position when first pressed and retracts the linear slide if pressed a second time.
 *     </li>
 *     <li>
 *         <em>gamepad2.triangle</em>: a toggle that moves the scoring subsystem to the high basket
 *         position when first pressed and retracts the linear slide if pressed a second time.
 *     </li>
 *     <li>
 *         <em>gamepad2.right_bumper</em>: move the scoring subsystem into a position where the
 *         linear slide can touch the low chamber bar, achieving an ascent level 1.
 *     </li>
 *     <li>
 *         <em>gamepad2.option</em>: move the scoring subsystem to a starting position, where the arm is
 *         all the way down and the linear slide is fully retracted.
 *     </li>
 *     <li>
 *         <em>gamepad2.dpad_left</em>: raises the arm on the scoring subsystem.
 *     </li>
 *     <li>
 *         <em>gamepad2.dpad_right</em>: lowers the arm on the scoring subsystem.
 *     </li>
 *     <li>
 *         <em>gamepad1.dpad_up</em>: a toggle that moves the scoring subsystem into the submersible
 *         at a position relatively far from the robot when first pressed and retracts the linear
 *         slide if pressed a second time.
 *     </li>
 *     <li>
 *         <em>gamepad1.dpad_down</em>: a toggle that moves the scoring subsystem into the submersible
 *         at a position relatively close to the robot when first pressed and retracts the linear
 *         slide if pressed a second time.
 *     </li>
 *     <li>
 *         <em>gamepad1.dpad_right</em>: retract the arm from the submersible to the neutral position.
 *     </li>
 *     <li>
 *         <em>gamepad1.triangle</em>: move the scoring subsystem to intake a specimen off the wall.
 *     </li>
 *     <li>
 *         <em>gamepad1.cross</em>: used to drop the sample off in the observation zone so the
 *         human player can turn it into a specimen.
 *     </li>
 * </ul>
 */
@Config
public class IntakeAndScoringSubsystemController implements SubsystemController {
    // Automate certain intake and scoring actions
    public static boolean AUTOMATE_HIGH_BASKET_SCORING = true;
    public static boolean AUTOMATE_SAMPLE_PICKUP = true;
    public static boolean AUTOMATE_DROPOFF_SAMPLE = true;

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
        //this.intakeRedSpecimenBehaviorTree = new IntakeRedSpecimenBehaviorTree(hardwareMap, telemetry);
    }

    /**
     * Updates the position of the arm, linear slide, wrist and claw based on user input using the
     * gamepads.
     *
     * @param gamepad1 Gamepad1
     * @param gamepad2 Gamepad2
     */
    public void execute(Gamepad gamepad1, Gamepad gamepad2) {
        // Preset positions for the arm and linear slide
        if (gamepad2.dpad_up && !previousGamepad2.dpad_up) {
            switch (state) {
                case INTAKE_SPECIMEN_OFF_WALL:
                case HIGH_CHAMBER_LOWERED:
                    intakeAndScoringSubsystem.moveToChamberHighScoringPosition();
                    state = State.HIGH_CHAMBER_SCORING;
                    break;
                case HIGH_CHAMBER_SCORING:
                    intakeAndScoringSubsystem.moveToChamberHighLoweredPosition();
                    state = State.HIGH_CHAMBER_LOWERED;
                    break;
                case HIGH_BASKET_SCORING:
                    intakeAndScoringSubsystem.moveToBasketHighRaisedPosition();
                    state = State.HIGH_BASKET_POST_SCORING;
                    break;
                case HIGH_BASKET_PRE_SCORING:
                case HIGH_BASKET_POST_SCORING:
                    intakeAndScoringSubsystem.moveToBasketHighRetractedPosition();
                    state = State.HIGH_BASKET_POST_RETRACTED;
                    break;
                default:
                    intakeAndScoringSubsystem.moveToChamberHighScoringPosition();
                    state = State.HIGH_CHAMBER_SCORING;
                    break;
            }
        } else if (gamepad2.triangle && !previousGamepad2.triangle) {
            switch (state) {
                case NEUTRAL_POSITION:
                    intakeAndScoringSubsystem.moveToBasketHighRetractedPosition();
                    state = State.HIGH_BASKET_PRE_RETRACTED;
                    break;
                case HIGH_BASKET_PRE_RETRACTED:
                case HIGH_BASKET_POST_RETRACTED:
                    intakeAndScoringSubsystem.moveToBasketHighRaisedPosition();
                    state = State.HIGH_BASKET_PRE_SCORING;
                    break;
                case HIGH_BASKET_PRE_SCORING:
                    intakeAndScoringSubsystem.moveToBasketHighScoringPosition();
                    state = State.HIGH_BASKET_SCORING;
                    break;
                case HIGH_BASKET_CLAW_OPENED:
                case HIGH_BASKET_SCORING:
                    intakeAndScoringSubsystem.moveToBasketHighRaisedPosition();
                    state = State.HIGH_BASKET_POST_SCORING;
                    break;
                case HIGH_BASKET_POST_SCORING:
                    intakeAndScoringSubsystem.moveToBasketHighRetractedPosition();
                    state = State.HIGH_BASKET_POST_RETRACTED;
                    break;
                default:
                    intakeAndScoringSubsystem.moveToNeutralPosition();
                    state = State.NEUTRAL_POSITION;
            }
        } else if (gamepad2.cross && !previousGamepad2.cross) {
            switch (state) {
                case INTAKE_SPECIMEN_OFF_WALL:
                    intakeAndScoringSubsystem.moveToBasketLowRetractedPosition();
                    state = State.LOW_BASKET_RETRACTED;
                    break;
                case LOW_BASKET_RETRACTED:
                    intakeAndScoringSubsystem.moveToBasketLowScoringPosition();
                    state = State.LOW_BASKET_SCORING;
                    break;
                case HIGH_BASKET_PRE_SCORING:
                case HIGH_BASKET_POST_SCORING:
                    intakeAndScoringSubsystem.moveToBasketHighRetractedPosition();
                    state = State.HIGH_BASKET_POST_RETRACTED;
                    break;
                case HIGH_BASKET_SCORING:
                    intakeAndScoringSubsystem.moveToBasketHighRaisedPosition();
                    state = State.HIGH_BASKET_POST_SCORING;
                    break;
                default:
                    intakeAndScoringSubsystem.moveToBasketLowRetractedPosition();
                    state = State.LOW_BASKET_RETRACTED;
            }
        } else if (gamepad2.right_bumper && !previousGamepad2.right_bumper) {
            switch (state) {
                case INTAKE_SPECIMEN_OFF_WALL:
                    intakeAndScoringSubsystem.moveToAscentLevelOne();
                    state = State.ASCENT_LEVEL_ONE;
                    break;
                case ASCENT_LEVEL_ONE:
                    intakeAndScoringSubsystem.moveToNeutralPosition();
                    state = State.NEUTRAL_POSITION;
                    break;
                case HIGH_BASKET_PRE_SCORING:
                case HIGH_BASKET_POST_SCORING:
                    intakeAndScoringSubsystem.moveToBasketHighRetractedPosition();
                    state = State.HIGH_BASKET_POST_RETRACTED;
                    break;
                case HIGH_BASKET_SCORING:
                    intakeAndScoringSubsystem.moveToBasketHighRaisedPosition();
                    state = State.HIGH_BASKET_POST_SCORING;
                    break;
                default:
                    intakeAndScoringSubsystem.moveToAscentLevelOne();
                    state = State.ASCENT_LEVEL_ONE;
            }
        } else if (gamepad2.options) {
            switch (state) {
                case NEUTRAL_POSITION:
                    intakeAndScoringSubsystem.moveToStartPosition();
                    state = State.START_POSITION;
                    break;
                case HIGH_BASKET_PRE_SCORING:
                case HIGH_BASKET_POST_SCORING:
                    intakeAndScoringSubsystem.moveToBasketHighRetractedPosition();
                    state = State.HIGH_BASKET_POST_RETRACTED;
                    break;
                case HIGH_BASKET_SCORING:
                    intakeAndScoringSubsystem.moveToBasketHighRaisedPosition();
                    state = State.HIGH_BASKET_POST_SCORING;
                    break;
                case START_POSITION:
                    // NO-OP
                    break;
                default:
                    intakeAndScoringSubsystem.moveToNeutralPosition();
                    state = State.NEUTRAL_POSITION;
            }
        } else if (gamepad2.dpad_left && !previousGamepad2.dpad_left) {
            intakeAndScoringSubsystem.raiseArm();
        } else if (gamepad2.dpad_right && !previousGamepad2.dpad_right) {
            intakeAndScoringSubsystem.lowerArm();
        } else if (gamepad1.dpad_up && !previousGamepad1.dpad_up) {
            switch (state) {
                case NEUTRAL_POSITION:
                    intakeAndScoringSubsystem.moveToIntakeFarAboveSamplePosition();
                    state = State.INTAKE_FAR_ABOVE_SAMPLE;
                    break;
                case INTAKE_FAR_ABOVE_SAMPLE:
                    intakeAndScoringSubsystem.openClaw();
                    if (Robot.getInstance().claw.isAtTarget() && !USE_WRIST_ORIENTATION) {
                        intakeAndScoringSubsystem.moveToIntakeFarLoweredPosition();
                        state = State.INTAKE_FAR_LOWERED_TO_SAMPLE;
                    } else {
                        state = State.INTAKE_FAR_CLAW_OPENED;
                    }
                    break;
                case INTAKE_FAR_CLAW_OPENED:
                    intakeAndScoringSubsystem.moveToIntakeFarLoweredPosition();
                    state = State.INTAKE_FAR_LOWERED_TO_SAMPLE;
                    break;
                case INTAKE_AUTO_GRAB_FAILED:
                    intakeAndScoringSubsystem.moveToIntakeFarLoweredPosition();
                    state = State.INTAKE_FAR_LOWERED_TO_SAMPLE;
                    break;
                case INTAKE_AUTO_ORIENTED:
                    intakeAndScoringSubsystem.moveToIntakeFarLoweredPosition();
                    state = State.INTAKE_FAR_LOWERED_TO_SAMPLE;
                    break;
                case INTAKE_FAR_LOWERED_TO_SAMPLE:
                    intakeAndScoringSubsystem.closeClaw();
                    state = State.INTAKE_FAR_CLAW_CLOSED;
                    break;
                case INTAKE_FAR_CLAW_CLOSED:
                    intakeAndScoringSubsystem.moveToIntakeFarAboveSamplePosition();
                    state = State.INTAKE_FAR_ABOVE_SAMPLE;
                     break;
                case HIGH_BASKET_PRE_SCORING:
                case HIGH_BASKET_POST_SCORING:
                    intakeAndScoringSubsystem.moveToBasketHighRetractedPosition();
                    state = State.HIGH_BASKET_POST_RETRACTED;
                    break;
                case HIGH_BASKET_SCORING:
                    intakeAndScoringSubsystem.moveToBasketHighRaisedPosition();
                    state = State.HIGH_BASKET_POST_SCORING;
                    break;
                default:
                    intakeAndScoringSubsystem.moveToNeutralPosition();
                    state = State.NEUTRAL_POSITION;
            }
        } else if (gamepad1.dpad_down && !previousGamepad1.dpad_down) {
            switch (state) {
                case NEUTRAL_POSITION:
                    intakeAndScoringSubsystem.moveToIntakeCloseAboveSamplePosition();
                    state = State.INTAKE_CLOSE_ABOVE_SAMPLE;
                    break;
                case INTAKE_CLOSE_ABOVE_SAMPLE:
                    intakeAndScoringSubsystem.openClaw();
                    if (Robot.getInstance().claw.isAtTarget() && !USE_WRIST_ORIENTATION) {
                        intakeAndScoringSubsystem.moveToIntakeCloseLoweredPosition();
                        state = State.INTAKE_CLOSE_LOWERED_TO_SAMPLE;
                    } else {
                        state = State.INTAKE_CLOSE_CLAW_OPENED;
                    }
                    break;
                case INTAKE_CLOSE_CLAW_OPENED:
                    intakeAndScoringSubsystem.moveToIntakeCloseLoweredPosition();
                    state = State.INTAKE_CLOSE_LOWERED_TO_SAMPLE;
                    break;
                case INTAKE_AUTO_GRAB_FAILED:
                    intakeAndScoringSubsystem.moveToIntakeCloseLoweredPosition();
                    state = State.INTAKE_CLOSE_LOWERED_TO_SAMPLE;
                    break;
                case INTAKE_CLOSE_LOWERED_TO_SAMPLE:
                    intakeAndScoringSubsystem.closeClaw();
                    state = State.INTAKE_CLOSE_CLAW_CLOSED;
                    break;
                case INTAKE_AUTO_ORIENTED:
                    intakeAndScoringSubsystem.moveToIntakeCloseLoweredPosition();
                    state = State.INTAKE_CLOSE_LOWERED_TO_SAMPLE;
                    break;
                case INTAKE_CLOSE_CLAW_CLOSED:
                    intakeAndScoringSubsystem.moveToIntakeCloseAboveSamplePosition();
                    state = State.INTAKE_CLOSE_ABOVE_SAMPLE;
                    break;
                case HIGH_BASKET_PRE_SCORING:
                case HIGH_BASKET_POST_SCORING:
                    intakeAndScoringSubsystem.moveToBasketHighRetractedPosition();
                    state = State.HIGH_BASKET_POST_RETRACTED;
                    break;
                case HIGH_BASKET_SCORING:
                    intakeAndScoringSubsystem.moveToBasketHighRaisedPosition();
                    state = State.HIGH_BASKET_POST_SCORING;
                    break;
                default:
                    intakeAndScoringSubsystem.moveToNeutralPosition();
                    state = State.NEUTRAL_POSITION;
            }
        } else if (gamepad1.dpad_right && !previousGamepad1.dpad_right) {
            if (state == State.INTAKE_CLOSE_ABOVE_SAMPLE || state == State.INTAKE_FAR_ABOVE_SAMPLE) {
                intakeAndScoringSubsystem.moveToNeutralPosition();
                state = State.NEUTRAL_POSITION;
            }
        } else if (gamepad1.triangle && !previousGamepad1.triangle) {
            switch (state) {
                case INTAKE_SPECIMEN_OFF_WALL:
                    intakeAndScoringSubsystem.closeClaw();
                    state = State.INTAKE_SPECIMEN_CLAW_CLOSED;
                    break;
                case INTAKE_SPECIMEN_CLAW_CLOSED:
                    intakeAndScoringSubsystem.moveToIntakeSpecimenRaisedPosition();
                    state = State.INTAKE_SPECIMEN_RAISED;
                    break;
                case INTAKE_SPECIMEN_RAISED:
                    intakeAndScoringSubsystem.moveToIntakeSpecimenOffWallPosition();
                    state = State.INTAKE_SPECIMEN_OFF_WALL;
                    break;
                case HIGH_BASKET_PRE_SCORING:
                case HIGH_BASKET_POST_SCORING:
                    intakeAndScoringSubsystem.moveToBasketHighRetractedPosition();
                    state = State.HIGH_BASKET_POST_RETRACTED;
                    break;
                case HIGH_BASKET_SCORING:
                    intakeAndScoringSubsystem.moveToBasketHighRaisedPosition();
                    state = State.HIGH_BASKET_POST_SCORING;
                    break;
                default:
                    intakeAndScoringSubsystem.moveToIntakeSpecimenOffWallPosition();
                    state = State.INTAKE_SPECIMEN_OFF_WALL;
            }
        } else if (gamepad1.cross && !previousGamepad1.cross) {
            switch (state) {
                case NEUTRAL_POSITION:
                    intakeAndScoringSubsystem.moveToDropOffSamplePosition();
                    state = State.DROP_OFF_SPECIMEN;
                    break;
                case DROP_OFF_SPECIMEN:
                    intakeAndScoringSubsystem.openClaw();
                    state = State.DROP_OFF_SPECIMEN_CLAW_OPEN;
                    break;
                case DROP_OFF_SPECIMEN_CLAW_OPEN:
                    intakeAndScoringSubsystem.moveToNeutralPosition();
                    state = State.NEUTRAL_POSITION;
                    break;
                case HIGH_BASKET_PRE_SCORING:
                case HIGH_BASKET_POST_SCORING:
                    intakeAndScoringSubsystem.moveToBasketHighRetractedPosition();
                    state = State.HIGH_BASKET_POST_RETRACTED;
                    break;
                case HIGH_BASKET_SCORING:
                    intakeAndScoringSubsystem.moveToBasketHighRaisedPosition();
                    state = State.HIGH_BASKET_POST_SCORING;
                    break;
                default:
                    intakeAndScoringSubsystem.moveToNeutralPosition();
                    state = State.NEUTRAL_POSITION;
            }
        }

        // Automate opening the claw, dropping the arm down, grabbing a sample, and raising the arm
        if (AUTOMATE_SAMPLE_PICKUP) {
            // Close intake position movements
            if (state == State.INTAKE_CLOSE_CLAW_OPENED && intakeAndScoringSubsystem.isAtTarget()) {
                intakeAndScoringSubsystem.moveToIntakeCloseLoweredPosition();
                state = State.INTAKE_CLOSE_LOWERED_TO_SAMPLE;
            }
            if (state == State.INTAKE_CLOSE_LOWERED_TO_SAMPLE && intakeAndScoringSubsystem.isAtTarget()) {
                intakeAndScoringSubsystem.closeClaw();
                state = State.INTAKE_CLOSE_CLAW_CLOSED;
            }
            if (state == State.INTAKE_CLOSE_CLAW_CLOSED && intakeAndScoringSubsystem.isAtTarget()) {
                intakeAndScoringSubsystem.moveToIntakeCloseAboveSamplePosition();
                state = State.INTAKE_CLOSE_ABOVE_SAMPLE;
            }

            // Far intake position movements
            if (state == State.INTAKE_FAR_CLAW_OPENED && intakeAndScoringSubsystem.isAtTarget()) {
                intakeAndScoringSubsystem.moveToIntakeCloseLoweredPosition();
                state = State.INTAKE_FAR_LOWERED_TO_SAMPLE;
            }
            if (state == State.INTAKE_FAR_LOWERED_TO_SAMPLE && intakeAndScoringSubsystem.isAtTarget()) {
                intakeAndScoringSubsystem.closeClaw();
                state = State.INTAKE_FAR_CLAW_CLOSED;
            }
            if (state == State.INTAKE_FAR_CLAW_CLOSED && intakeAndScoringSubsystem.isAtTarget()) {
                intakeAndScoringSubsystem.moveToIntakeCloseAboveSamplePosition();
                state = State.INTAKE_FAR_ABOVE_SAMPLE;
            }

            // Automate intake off wall
            if (state == State.INTAKE_SPECIMEN_CLAW_CLOSED && intakeAndScoringSubsystem.isAtTarget()) {
                intakeAndScoringSubsystem.moveToIntakeSpecimenRaisedPosition();
                state = State.INTAKE_SPECIMEN_RAISED;
            }
        }

        // Automate dropping off a sample
        if (AUTOMATE_DROPOFF_SAMPLE) {
            if (state == State.DROP_OFF_SPECIMEN && intakeAndScoringSubsystem.isAtTarget()) {
                intakeAndScoringSubsystem.openClaw();
                state = State.DROP_OFF_SPECIMEN_CLAW_OPEN;
            }
            if (state == State.DROP_OFF_SPECIMEN_CLAW_OPEN && intakeAndScoringSubsystem.isAtTarget()) {
                intakeAndScoringSubsystem.moveToNeutralPosition();
                state = State.NEUTRAL_POSITION;
            }
        }

        // Automate scoring in the high basket. For now, don't auto-retract the linear slide as it
        // might get caught on baskets on descent if the robot is driven too close to the baskets.
        if (AUTOMATE_HIGH_BASKET_SCORING) {
            if (state == State.HIGH_BASKET_PRE_SCORING && intakeAndScoringSubsystem.isAtTarget()) {
                intakeAndScoringSubsystem.moveToBasketHighScoringPosition();
                state = State.HIGH_BASKET_SCORING;
            }
            if (state == State.HIGH_BASKET_SCORING && intakeAndScoringSubsystem.isAtTarget()) {
                intakeAndScoringSubsystem.openClaw();
                state = State.HIGH_BASKET_CLAW_OPENED;
            }
            if (state == State.HIGH_BASKET_CLAW_OPENED && intakeAndScoringSubsystem.isAtTarget()) {
                intakeAndScoringSubsystem.moveToBasketHighRaisedPosition();
                state = State.HIGH_BASKET_POST_SCORING;
            }
            if (state == State.HIGH_BASKET_POST_RETRACTED && intakeAndScoringSubsystem.isAtTarget()) {
                intakeAndScoringSubsystem.moveToNeutralPosition();
                state = State.NEUTRAL_POSITION;
            }
        }

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

        telemetry.addData("[IAS-C] state", state);

        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }

    /**
     * State of the intake and scoring subsystem
     */
    enum State {
        ASCENT_LEVEL_ONE,
        DROP_OFF_SPECIMEN,
        DROP_OFF_SPECIMEN_CLAW_OPEN,
        HIGH_BASKET_CLAW_OPENED,
        HIGH_BASKET_PRE_SCORING,
        HIGH_BASKET_PRE_RETRACTED,
        HIGH_BASKET_POST_SCORING,
        HIGH_BASKET_POST_RETRACTED,
        HIGH_BASKET_SCORING,
        HIGH_CHAMBER_LOWERED,
        HIGH_CHAMBER_SCORING,
        INTAKE_CLOSE_ABOVE_SAMPLE,
        INTAKE_CLOSE_LOWERED_TO_SAMPLE,
        INTAKE_CLOSE_CLAW_CLOSED,
        INTAKE_CLOSE_CLAW_OPENED,
        INTAKE_FAR_ABOVE_SAMPLE,
        INTAKE_FAR_LOWERED_TO_SAMPLE,
        INTAKE_FAR_CLAW_CLOSED,
        INTAKE_FAR_CLAW_OPENED,
        INTAKE_AUTO_ORIENTED,
        INTAKE_AUTO_GRAB_FAILED,
        LOW_BASKET_RETRACTED,
        LOW_BASKET_SCORING,
        INTAKE_SPECIMEN_OFF_WALL,
        INTAKE_SPECIMEN_CLAW_CLOSED,
        INTAKE_SPECIMEN_RAISED,
        NEUTRAL_POSITION,
        START_POSITION,
    }
}
