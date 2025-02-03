package org.firstinspires.ftc.teamcode.ftc7083.action;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Runs an array of actions in parallel. The action completes running only when all actions have
 * completed.
 */
public class ParallelAction extends ActionExBase {
    private final List<Action> actions = new ArrayList<>();

    /**
     * Instantiates a ParallelAction that runs a set of actions in parallel.
     *
     * @param actions the actions to run in parallel
     */
    public ParallelAction(Action... actions) {
        this(Arrays.asList(actions));
    }

    /**
     * Instantiates a ParallelAction that runs a set of actions in parallel.
     *
     * @param actions the actions to run in parallel
     */
    public ParallelAction(List<Action> actions) {
        this.actions.addAll(actions);
    }

    /**
     * Runs each action in the set in parallel until every action has finished running.
     *
     * @param telemetryPacket the telemetry to use for output.
     * @return <code>false</code> if all actions have finished running; <code>true</code> if any
     *         of the actions are still running.
     */
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        // Run each action in the list in sequence
        List<Boolean> actionRunning = new ArrayList<>();
        for (Action action : actions) {
            actions.removeIf(a -> !a.run(telemetryPacket));
        }

        return !actions.isEmpty();
    }

    @Override
    public void preview(@NonNull Canvas fieldOverlay) {
        for (Action a : actions) {
            a.preview(fieldOverlay);
        }
    }
}
