package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayDeque;
import java.util.Queue;

@Config
public class Limelight extends SubsystemBase {

    public static double LL_ANGLE_WITH_VERTICAL = 20.7;
    public static double LL_HEIGHT = 15.5;

    private final Limelight3A limelight;
    private final Telemetry telemetry;
    private LLResult result;
    private LLStatus status;

    private final int YELLOW_SAMPLE_COLOR_PIPELINE = 0;
    private final int RED_SAMPLE_COLOR_PIPELINE = 1;
    private final int BLUE_SAMPLE_COLOR_PIPELINE = 2;
    private final int APRIL_TAG_PIPELINE = 3;
    public static int MAX_COLOR_PIPELINE = 2;
    private static int NUM_SAMPLES_TO_AVERAGE = 3;
    private static final double SAMPLE_HEIGHT_INCHES = 1.1;
    private static final double WALL_HEIGHT_INCHES = 6.5;
    private static double LL_DISTANCE_FROM_ARM_AXEL = 2;

    private final Queue<Double> TySamples = new ArrayDeque<>();


    /**
     * Creates a limelight
     *
     * @param hardwareMap hardwareMap
     * @param telemetry   telemetry
     */
    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this.limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        this.telemetry = telemetry;
        configureLimelight();
    }

    @Override
    public void execute() {

        if (getTy() != null) {
            double Ty = (double)getTy();
            TySamples.add(Ty);

            telemetry.addData("[Limelight]","Added a Ty");
            telemetry.update();
            if (TySamples.size() > NUM_SAMPLES_TO_AVERAGE) {
                TySamples.remove();
            }
        }
    }

    /**
     * Configures the limelight
     */
    private void configureLimelight() {
        detectYellow();
        limelight.setPollRateHz(250);
        limelight.start();
    }

    /**
     * Gets the latest result from the set pipeline
     */
    public LLResult getResult() {
        result = limelight.getLatestResult();
        return result;
    }

    private void getStatus() {
        status = limelight.getStatus();
    }

    /**
     * Gets the Tx angle if the current pipeline is color
     *
     * @return the Tx angle
     */
    public Object getTx() {
        getStatus();
        getResult();

        if (result != null && (status.getPipelineIndex() <= MAX_COLOR_PIPELINE) && result.getTx() != 0.0) {
            telemetry.addData("[Limelight]","Found a Tx");
            telemetry.update();

            return result.getTx();
        } else {
            return null;
        }
    }

    /**
     * Gets the Ty angle from the principle pixel if the current pipeline is color
     *
     * @return the Ty angle
     */
    private Object getTy() {
        getStatus();
        getResult();

        if (result != null && (status.getPipelineIndex() <= MAX_COLOR_PIPELINE)) {
            return result.getTyNC();
        } else {
            return null;
        }
    }

    /**
     * Gets the x distance from the shoulder to the target
     * if the current pipeline is color.
     *
     * @return the distance to the target
     */
    public Object getDistance(TargetPosition position) {
        double xDistance;
        double retryCount = 0;
        double filteredTy;
        final double MAX_RETRIES = 20;
        double goalHeightInches;


        if (position == TargetPosition.WALL) {
            goalHeightInches = WALL_HEIGHT_INCHES;
        } else {
            goalHeightInches = SAMPLE_HEIGHT_INCHES;
        }


        while (TySamples.size() < NUM_SAMPLES_TO_AVERAGE && retryCount < MAX_RETRIES) {
            execute();
            retryCount++;
        }

        if (!TySamples.stream().mapToDouble(a -> a).average().isPresent()) {
            return null;
        } else {
            filteredTy = TySamples.stream().mapToDouble(a -> a).average().getAsDouble();
        }

        double angleToGoalDegrees = 90 - LL_ANGLE_WITH_VERTICAL + filteredTy;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        xDistance = (LL_HEIGHT - goalHeightInches) * Math.tan(angleToGoalRadians);
        return xDistance + LL_DISTANCE_FROM_ARM_AXEL;
    }


    /**
     * Sets the pipeline to detect yellow samples.
     */
    public void detectYellow() {
        limelight.pipelineSwitch(YELLOW_SAMPLE_COLOR_PIPELINE);
        TySamples.clear();
    }

    /**
     * Sets the pipeline to detect red samples.
     */
    public void detectRed() {
        limelight.pipelineSwitch(RED_SAMPLE_COLOR_PIPELINE);
        TySamples.clear();
    }

    /**
     * Sets the pipeline to detect blue samples.
     */
    public void detectBlue() {
        limelight.pipelineSwitch(BLUE_SAMPLE_COLOR_PIPELINE);
        TySamples.clear();
    }

    /**
     * Setts the pipeline to detect aprilTags.
     */
    public void detectAprilTags() {
        limelight.pipelineSwitch(APRIL_TAG_PIPELINE);
        TySamples.clear();
    }

    @NonNull
    @Override
    public String toString() {
        return "Limelight{" +
                "status=" + status +
                ", result=" + result +
                '}';
    }

    public enum TargetPosition {
        SUBMERSIBLE,
        WALL;
    }

}
