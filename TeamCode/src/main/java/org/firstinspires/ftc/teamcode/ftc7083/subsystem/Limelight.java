package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Limelight extends SubsystemBase {

    public static double LL_ANGLE_WITH_VERTICAL = 15.2;
    public static double LL_HEIGHT = 15.3;

    private final Limelight3A limelight;
    private final Telemetry telemetry;
    private LLResult result;
    private LLStatus status;

    private final int YELLOW_SAMPLE_COLOR_PIPELINE = 0;
    private final int RED_SAMPLE_COLOR_PIPELINE = 1;
    private final int BLUE_SAMPLE_COLOR_PIPELINE = 2;
    private final int APRIL_TAG_PIPELINE = 3;
    public static int MAX_COLOR_PIPELINE = 2;
    private final double LIMELIGHT_LATERAL_OFFSET = 4.5;
    private static final double SAMPLE_HEIGHT_INCHES = 1.1;
    private static final double WALL_HEIGHT_INCHES = 10;
    private static double LL_DISTANCE_FROM_BACK_OF_SLIDE = -1.2; // had -1.6 previously (too close) had -2.5 (too far)


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
    public Double getTx() {
        getStatus();
        getResult();

        if (result != null && (status.getPipelineIndex() <= MAX_COLOR_PIPELINE) && result.getTx() != 0.0) {
            telemetry.addData("[Limelight]","Found a Tx");
            telemetry.update();

            double distance = getDistance(TargetPosition.SUBMERSIBLE) - LL_DISTANCE_FROM_BACK_OF_SLIDE;
            double detectedAngle = result.getTx() * (Math.PI/180);
            double horizontalDetectionDisplacement = distance * Math.tan(detectedAngle);
            double actualHorizontalDisplacement = LIMELIGHT_LATERAL_OFFSET + horizontalDetectionDisplacement;
            double adjustedAngle = Math.tan(actualHorizontalDisplacement/distance);
            return adjustedAngle * (180/Math.PI); //convert back to degrees
        } else {
            return null;
        }
    }

    /**
     * Gets the Ty angle from the principle pixel if the current pipeline is color
     *
     * @return the Ty angle
     */
    private Double getTy() {
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
    public Double getDistance(TargetPosition position) {
        double xDistance;
        Double Ty = getTy();
        double goalHeightInches;


        if (position == TargetPosition.WALL) {
            goalHeightInches = WALL_HEIGHT_INCHES;
        } else {
            goalHeightInches = SAMPLE_HEIGHT_INCHES;
        }


        if (Ty != null) {
            double angleToGoalDegrees = 90 - LL_ANGLE_WITH_VERTICAL + Ty;
            double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

            xDistance = (LL_HEIGHT - goalHeightInches) * Math.tan(angleToGoalRadians);
            return xDistance - LL_DISTANCE_FROM_BACK_OF_SLIDE;
        } else {
            return null;
        }
    }


    /**
     * Sets the pipeline to detect yellow samples.
     */
    public void detectYellow() {
        limelight.pipelineSwitch(YELLOW_SAMPLE_COLOR_PIPELINE);
    }

    /**
     * Sets the pipeline to detect red samples.
     */
    public void detectRed() {
        limelight.pipelineSwitch(RED_SAMPLE_COLOR_PIPELINE);
    }

    /**
     * Sets the pipeline to detect blue samples.
     */
    public void detectBlue() {
        limelight.pipelineSwitch(BLUE_SAMPLE_COLOR_PIPELINE);
    }

    /**
     * Setts the pipeline to detect aprilTags.
     */
    public void detectAprilTags() {
        limelight.pipelineSwitch(APRIL_TAG_PIPELINE);
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
