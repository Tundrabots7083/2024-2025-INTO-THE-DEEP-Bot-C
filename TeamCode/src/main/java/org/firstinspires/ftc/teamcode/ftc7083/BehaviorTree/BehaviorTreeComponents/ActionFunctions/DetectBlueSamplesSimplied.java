package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.ActionFunctions;


import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Limelight;

public class DetectBlueSamplesSimplied implements ActionFunction {
    private Limelight limelight;
    private Telemetry telemetry;
    private LLResult result;
    private Status lastStatus = Status.FAILURE;
    private int count = 0;

    public DetectBlueSamplesSimplied(Telemetry telemetry, Limelight limelight) {
        this.limelight = limelight;
        this.telemetry = telemetry;
    }

    public Status perform(BlackBoardSingleton blackBoard) {

        if (lastStatus == Status.SUCCESS) {
            return Status.SUCCESS;
        }

        telemetry.addData("[DetectBlueSamples]", " perform count: %d", count);
        telemetry.update();
        count++;

        limelight.detectBlue();

        result = limelight.getResult();

        if (result != null) {
            blackBoard.setValue("BlueSampleDetected", true);
            lastStatus = Status.SUCCESS;
            return Status.SUCCESS;
        } else {
            blackBoard.setValue("BlueSampleDetected", false);
        }

        if (count > 5) {
            lastStatus = Status.SUCCESS;
            return Status.SUCCESS;
        } else if (count < 5) {
            return Status.FAILURE;
        }

        return Status.RUNNING;
    }
}
