package org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.ActionFunctions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.GlobalShutterCamera;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.processors.SampleProcessor;

public class DetectSampleOrientation implements ActionFunction {
    GlobalShutterCamera globalShutterCamera;
    private Telemetry telemetry;
    private int count=0;
    protected Status lastStatus = Status.FAILURE;

    public DetectSampleOrientation(Telemetry telemetry, GlobalShutterCamera globalShutterCamera) {
        this.globalShutterCamera = globalShutterCamera;
        this.telemetry = telemetry;
    }

    public Status perform(BlackBoardSingleton blackBoard) {
        Status status;

        if(lastStatus == Status.SUCCESS){
            return lastStatus;
        }

        telemetry.addData("[DetectSampleOrientation]", " perform count: %d", count);
        telemetry.update();
        //count++;

        SampleProcessor sampleProcessor = new SampleProcessor();
         Double yellowAngle = sampleProcessor.execute(globalShutterCamera.getYellowDetections());
         Double blueAngle = sampleProcessor.execute(globalShutterCamera.getBlueDetections());
         //Double redAngle = sampleProcessor.execute(globalShutterCamera.getRedDetections());

         telemetry.addData("Yellow Angle", yellowAngle);
        // telemetry.addData("Blue Angle", blueAngle);
         //telemetry.addData("Red Angle", redAngle);

        if (yellowAngle != null) {
            yellowAngle -= 90;
            blackBoard.setValue("SampleAngle", yellowAngle);
            status = Status.SUCCESS;

        } else {
            blackBoard.setValue("SampleAngle", null);

            telemetry.addData("[DetectSampleOrientation]", "Didn't detect anything");
            telemetry.update();

            if (count >= 50) {
                status = Status.FAILURE;
            } else {
                status = Status.RUNNING;
            }
        }

        lastStatus = status;
        return status;
    }

}
