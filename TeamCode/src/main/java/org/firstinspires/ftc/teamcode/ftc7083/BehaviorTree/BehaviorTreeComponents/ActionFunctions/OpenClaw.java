package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;

public class OpenClaw implements ActionFunction {

    Robot robot;
    Telemetry telemetry;

    protected Status lastStatus = Status.FAILURE;
    protected int runCount = 0;
    private TelemetryPacket tp = new TelemetryPacket();
    private boolean running = true;


    public OpenClaw(Telemetry telemetry, Robot robot) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public Status perform(BlackBoardSingleton blackBoard) {
        Status status = Status.RUNNING;

        if(runCount == 0) {
            running = robot.claw.actionOpenClaw().run(tp);
        }


        if(lastStatus == Status.SUCCESS){
            return lastStatus;
        }


        if (!running) {
            status = Status.SUCCESS;
        }

        runCount++;
        lastStatus = status;

        return status;
    }
}
