package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.PIDControllerImpl;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.MecanumDrive;

@Config
public class SquareBotWithSample implements ActionFunction {
    MecanumDrive mecanumDrive;
    Telemetry telemetry;

    public static double KP = 0.01;
    public static double KI = 0.1;
    public static double KD = 0.0;
    public static double TOLERABLE_ERROR = 0.4; // degrees
    public static double INTAKE_ANGLE = 0.0;

    private final PIDControllerImpl pidController;

    protected Status lastStatus = Status.FAILURE;
    protected int runCount = 0;

    public SquareBotWithSample(Telemetry telemetry, MecanumDrive mecanumDrive) {
        this.mecanumDrive = mecanumDrive;
        this.telemetry = telemetry;
        pidController = new PIDControllerImpl(KP, KI, KD);
        pidController.reset();
    }

    public Status perform(BlackBoardSingleton blackBoard) {
        Status status = Status.RUNNING;
        double turnError;

        if (lastStatus == Status.SUCCESS) {
            telemetry.addData("[SquareBotWithSample] status:", lastStatus);
            telemetry.update();
            return lastStatus;
        }


        if (blackBoard.getValue("Tx") != null) {
            turnError = (double) blackBoard.getValue("Tx");
        } else {
            status = Status.FAILURE;
            blackBoard.setValue("BotIsSquaredWithSample",false);
            return status;
        }

        if (Math.abs(turnError) <= TOLERABLE_ERROR) {
            mecanumDrive.turn(0.0);
            status = Status.SUCCESS;
            lastStatus = status;
            telemetry.addData("[SquareBotWithSample] status:", status);
            telemetry.update();
            if (blackBoard.getValue("BotIsInIntakeRange") != null) {
                blackBoard.setValue("BotIsSquaredWithSample", true);
            } else {
                blackBoard.setValue("BotIsSquaredWithSample", false);
            }
            return status;
        }

        double turnPower = pidController.calculate(INTAKE_ANGLE,turnError);

        mecanumDrive.turn(turnPower);

        runCount++;
        lastStatus = status;

        telemetry.addData("[SquareBotWithSample] status:", status);
        telemetry.update();

        return status;
    }
}
