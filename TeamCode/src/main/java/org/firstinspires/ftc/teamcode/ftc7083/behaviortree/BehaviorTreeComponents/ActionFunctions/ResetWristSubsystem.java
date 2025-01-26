package org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.ActionFunctions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Wrist;

public class ResetWristSubsystem implements ActionFunction  {
    IntakeAndScoringSubsystem intakeAndScoringSubsystem;
    Telemetry telemetry;

    protected Status lastStatus = Status.FAILURE;
    protected int runCount = 0;

    public ResetWristSubsystem (Telemetry telemetry,IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
        this.intakeAndScoringSubsystem =intakeAndScoringSubsystem;
        this.telemetry = telemetry;
    }


    public Status perform(BlackBoardSingleton blackBoard) {
        Status status = Status.RUNNING;

        if(lastStatus == Status.SUCCESS){
            return lastStatus;
        }

        intakeAndScoringSubsystem.setWristRoll(0);

        if(runCount > 250) {
            status = Status.SUCCESS;
        }

        runCount++;
        lastStatus = status;

        return status;
    }
}
