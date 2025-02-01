package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class Selector extends Node {
    private List<Node> children;
    protected Telemetry telemetry;

    public Selector(List<Node> children, Telemetry telemetry) {
        this.children = children;
        this.telemetry = telemetry;

    }

    @Override
    public Status execute(BlackBoardSingleton globalStore) {
        for (Node child : children) {
            Status status = child.execute(globalStore);
            telemetry.addData("Selector", "Selector status: %b",status == Status.SUCCESS);
            telemetry.update();
            if (status == Status.SUCCESS) {
                return Status.SUCCESS;
            } else if (status == Status.RUNNING) {
                return Status.RUNNING;
            }
        }
        return Status.FAILURE;
    }
}
