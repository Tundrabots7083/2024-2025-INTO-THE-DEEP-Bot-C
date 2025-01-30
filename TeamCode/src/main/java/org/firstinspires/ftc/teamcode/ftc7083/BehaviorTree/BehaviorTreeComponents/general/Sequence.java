package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Sequence extends Node {
    private List<Node> children;
    protected Telemetry telemetry;

    public Sequence(List<Node> children, Telemetry telemetry) {
        this.children = children;
        this.telemetry = telemetry;
    }

    @Override
    public Status execute(BlackBoardSingleton globalStore) {
        for (Node child : children) {
            Status status = child.execute(globalStore);
          //  opMode.telemetry.addData("Sequence", "Sequence execute result: %b num children = %d",status==Status.SUCCESS, children.stream().count());
          //  opMode.telemetry.update();
            if (status == Status.FAILURE) {
                return Status.FAILURE;
            } else if (status == Status.RUNNING) {
                return Status.RUNNING;
            }
        }
       // opMode.telemetry.addData("Sequence", "Sequence execute ---------------");
       // opMode.telemetry.update();
        return Status.SUCCESS;
    }
}
