package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;

public class LowerArmToSubmersibleSample implements ActionFunction {

    IntakeAndScoringSubsystem intakeAndScoringSubsystem;
    Telemetry telemetry;

    protected Status lastStatus = Status.FAILURE;
    protected int runCount = 0;

    public LowerArmToSubmersibleSample(Telemetry telemetry, IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
        this.intakeAndScoringSubsystem = intakeAndScoringSubsystem;
        this.telemetry = telemetry;
    }

    public Status perform(BlackBoardSingleton blackBoard) {
        Status status = Status.RUNNING;
        double xDistance;

        if(lastStatus == Status.SUCCESS){
            return lastStatus;
        }



        if(blackBoard.getValue("xDistanceToSample") != null) {
            xDistance = (double)blackBoard.getValue("xDistanceToSample");
        } else {
            status = Status.FAILURE;
            return status;
        }

        intakeAndScoringSubsystem.moveToPosition(xDistance,1.3);
        intakeAndScoringSubsystem.execute();

        if(intakeAndScoringSubsystem.isAtTarget()) {
            status = Status.SUCCESS;
        }

        runCount++;
        lastStatus = status;

        return status;
    }
}
