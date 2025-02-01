package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Wrist;

public class TurnWrist implements ActionFunction  {
   // IntakeAndScoringSubsystem intakeAndScoringSubsystem;
    Wrist wrist;
    Telemetry telemetry;
    ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    protected Status lastStatus = Status.FAILURE;
    protected int runCount = 0;
/*
    public TurnWrist (Telemetry telemetry,IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
        this.intakeAndScoringSubsystem =intakeAndScoringSubsystem;
        this.telemetry = telemetry;
    }
    */
    public TurnWrist (Telemetry telemetry,Wrist wrist) {
        this.wrist =wrist;
        this.telemetry = telemetry;
        this.time.reset();
    }

    public Status perform(BlackBoardSingleton blackBoard) {
        Status status = Status.RUNNING;

        if(lastStatus == Status.SUCCESS){
            return lastStatus;
        }

        Double sampleAngle =  (Double) blackBoard.getValue("SampleAngle");

        if(runCount == 0 && sampleAngle != null) {
                wrist.setRollDegrees(-sampleAngle);
                telemetry.addData("[TurnWrist] Sample angle", sampleAngle);
                telemetry.update();
            }

        if(time.time() >= 100) {
            status = Status.SUCCESS;
        }

        runCount++;
        lastStatus = status;

        return status;
    }
}
