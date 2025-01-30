package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general;

import java.util.ArrayList;
import java.util.List;

class Parallel extends Node {
    private final List<Node> children;

    Parallel(List<Node> children) {
        this.children = children;
    }

    @Override
    public Status execute(BlackBoardSingleton globalStore) {
        List<Status> childStatuses = new ArrayList<>();

        for (Node child : children) {
            Status status = child.execute(globalStore);
            childStatuses.add(status);
        }

        // Check if any child succeeded
        if (childStatuses.contains(Status.SUCCESS)) {
            return Status.SUCCESS;
        }
        // Check if any child is still running
        else if (childStatuses.contains(Status.RUNNING)) {
            return Status.RUNNING;
        }
        // All children failed
        else {
            return Status.FAILURE;
        }
    }
}
