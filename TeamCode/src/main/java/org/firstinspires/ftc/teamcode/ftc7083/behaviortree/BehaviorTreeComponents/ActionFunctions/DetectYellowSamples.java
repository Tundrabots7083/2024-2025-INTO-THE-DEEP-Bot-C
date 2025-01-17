package org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.ActionFunctions;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Limelight;

public class DetectYellowSamples implements ActionFunction {
    private Limelight limelight;
    private Telemetry telemetry;
    private Limelight.TargetPosition targetHeight;
    private int count=0;
    private double xDistance;
    private double Tx;

    public DetectYellowSamples(Telemetry telemetry, Limelight limelight, Limelight.TargetPosition targetHeight) {
        this.limelight = limelight;
        this.telemetry = telemetry;
        this.targetHeight = targetHeight;
    }

    public Status perform(BlackBoardSingleton blackBoard) {

        telemetry.addData("[DetectYellowSamples]"," perform count: %d", count);
        telemetry.update();
        count++;

        limelight.detectYellow();
        limelight.execute();

        LLResult result = limelight.getResult();

        if(result != null && limelight.getTx() != null && limelight.getDistance(targetHeight) != null){
            xDistance = (double)limelight.getDistance(targetHeight);
            Tx = (double)limelight.getTx();

            blackBoard.setValue("xDistanceToSample", xDistance);
            blackBoard.setValue("Tx",Tx);

            telemetry.addData("[DetectYellowSamples]","X-Distance to Sample: %f",xDistance);
            telemetry.addData("[DetectYellowSamples]","Tx-Angle from Sample: %f",Tx);
            telemetry.update();

            return Status.SUCCESS;
        } else {
            blackBoard.setValue("xDistanceToSample", null);
            blackBoard.setValue("Tx",null);

            telemetry.addData("[DetectYellowSamples]","Didn't detect anything");
            telemetry.update();

            if(count >= 25) {
                return Status.FAILURE;
            } else {
                return Status.RUNNING;
            }
        }
    }


}
