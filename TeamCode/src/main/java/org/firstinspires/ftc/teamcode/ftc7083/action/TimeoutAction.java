package org.firstinspires.ftc.teamcode.ftc7083.action;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

/**
 * Runs the action for a maximum number of time, completing if the action finishes running or the
 * timeout expires.
 */
public class TimeoutAction extends ActionExBase {
    private final Action action;
    private final WaitAction waitAction;

    /**
     * Instantiates a timeout action to run the specified action for at most the number of milliseconds
     * provided on input.
     *
     * @param action the action to run
     * @param millis the maximum number of milliseconds the action may run for
     */
    public TimeoutAction(Action action, long millis) {
        this.action = action;
        this.waitAction = new WaitAction(millis);
    }

    /**
     * Runs the action for a maximum amount of time.
     *
     * @param telemetryPacket the telemetry to use for output.
     * @return <code>false</code> if the action has completed execution or if the execution has
     *         timed out; <code>true</code> if the action has not completed execution and has not timed out
     */
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return action.run(telemetryPacket) && waitAction.run(telemetryPacket);
    }
}
