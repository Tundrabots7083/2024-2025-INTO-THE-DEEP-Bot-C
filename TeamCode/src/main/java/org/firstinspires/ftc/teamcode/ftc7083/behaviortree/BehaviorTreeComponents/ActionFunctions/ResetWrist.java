package org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.ActionFunctions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Wrist;

public class ResetWrist implements ActionFunction  {
   // IntakeAndScoringSubsystem intakeAndScoringSubsystem;
    Wrist wrist;
    Telemetry telemetry;

    protected Status lastStatus = Status.FAILURE;
    protected int runCount = 0;
/*
    public ResetWrist (Telemetry telemetry,IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
        this.intakeAndScoringSubsystem =intakeAndScoringSubsystem;
        this.telemetry = telemetry;
    }
    */
    public ResetWrist(Telemetry telemetry, Wrist wrist) {
        this.wrist =wrist;
        this.telemetry = telemetry;
    }

    public Status perform(BlackBoardSingleton blackBoard) {
        Status status = Status.RUNNING;

        if(lastStatus == Status.SUCCESS){
            return lastStatus;
        }

        wrist.setRollDegrees(0);

        if(runCount > 250) {
            status = Status.SUCCESS;
        }

        runCount++;
        lastStatus = status;

        return status;
    }
}
