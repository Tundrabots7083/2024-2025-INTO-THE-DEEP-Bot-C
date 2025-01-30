package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;

public class MoveToStartPosition implements ActionFunction {

    IntakeAndScoringSubsystem intakeAndScoringSubsystem;
    Telemetry telemetry;

    protected Status lastStatus = Status.FAILURE;
    protected int runCount = 0;

    public MoveToStartPosition(Telemetry telemetry, IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
        this.intakeAndScoringSubsystem =intakeAndScoringSubsystem;
        this.telemetry = telemetry;
    }

    public Status perform(BlackBoardSingleton blackBoard) {
        Status status;

        if(lastStatus == Status.SUCCESS){
            return lastStatus;
        }

        intakeAndScoringSubsystem.moveToStartPosition();
        intakeAndScoringSubsystem.execute();

        if (intakeAndScoringSubsystem.isAtTarget()) {
            status = Status.SUCCESS;
        } else {
            status = Status.RUNNING;
        }

        runCount++;
        lastStatus = status;

        return status;
    }
}
