package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general;

public class Conditional extends Node {
    private Condition condition;

    public Conditional(Condition condition) {
        this.condition = condition;
    }

    @Override
    public Status execute(BlackBoardSingleton blackBoard) {
        boolean result = condition.check(blackBoard);
        if (result) {
            return Status.SUCCESS;
        } else {
            return Status.FAILURE;
        }

        // return condition.check() ? Status.SUCCESS : Status.FAILURE;
    }
}
