package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;

public class CloseClaw implements ActionFunction {

    Robot robot;
    Telemetry telemetry;

    protected Status lastStatus = Status.FAILURE;
    protected int runCount = 0;
    private TelemetryPacket tp = new TelemetryPacket();
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public CloseClaw (Telemetry telemetry,Robot robot) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public Status perform(BlackBoardSingleton blackBoard) {
        Status status = Status.RUNNING;

        if(lastStatus == Status.SUCCESS){
            return lastStatus;
        }

        if(runCount == 0) {
            timer.reset();
        }

        robot.intakeAndScoringSubsystem.actionCloseClaw().run(tp);

        if (timer.time() >= 75) {
            status = Status.SUCCESS;
        }


        runCount++;
        lastStatus = status;

        return status;
    }
}
