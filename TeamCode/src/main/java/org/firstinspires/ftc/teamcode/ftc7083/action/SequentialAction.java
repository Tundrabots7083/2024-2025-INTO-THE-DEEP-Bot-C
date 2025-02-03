package org.firstinspires.ftc.teamcode.ftc7083.action;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.LinkedList;
import java.util.Arrays;
import java.util.List;

/**
 * Runs the set of actions sequentially. This action only completes when the last action finishes
 * running.
 */
public class SequentialAction extends ActionExBase {
    private final List<Action> actions = new LinkedList<>();

    /**
     * Instantiates an action that runs the set of actions sequential.
     *
     * @param actions the set of actions to run sequentially.
     */
    public SequentialAction(Action... actions) {
        this(Arrays.asList(actions));
    }

    /**
     * Instantiates an action that runs the set of actions sequential.
     *
     * @param actions the set of actions to run sequentially.
     */
    public SequentialAction(List<Action> actions) {
        this.actions.addAll(actions);
    }

    /**
     * Runs the actions sequentially, only completing when every action in the set has completed
     * running.
     *
     * @param telemetryPacket the telemetry to use for output.
     * @return <code>false</code> if all actions have completed running; <code>true</code> if any
     *         action is still running.
     */
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (actions.isEmpty()) {
            return false;
        }

        if (actions.get(0).run(telemetryPacket)) {
            return true;
        } else {
            actions.remove(0);
            return run(telemetryPacket);
        }
    }

    @Override
    public void preview(@NonNull Canvas fieldOverlay) {
        for (Action a : actions) {
            a.preview(fieldOverlay);
        }
    }
}
