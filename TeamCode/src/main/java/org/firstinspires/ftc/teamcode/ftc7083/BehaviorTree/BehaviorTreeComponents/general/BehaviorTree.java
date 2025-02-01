package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTree.BehaviorTreeComponents.general;


public class BehaviorTree {
    private Node root;
    private BlackBoardSingleton blackBoard;

    public BehaviorTree(Node root, BlackBoardSingleton blackBoard) {
        this.root = root;
        this.blackBoard = blackBoard;
    }

    public Status tick() {
        return root.execute(blackBoard);
    }
}
