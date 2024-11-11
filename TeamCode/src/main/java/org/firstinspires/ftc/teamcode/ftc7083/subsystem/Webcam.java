package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A webcam used to create and obtain data from a vision processor and detect the visible
 * April Tags.
 */
@Config
public class Webcam extends SubsystemBase {
    // List of pre-calibrated camera resolutions supported by the Logi 920 webcam
    public static final List<Size> SUPPORTED_CAMERA_RESOLUTIONS = Arrays.asList(
            new Size(640, 480),
            new Size(640, 360),
            new Size(800, 448),
            new Size(800, 600),
            new Size(864, 480),
            new Size(1920, 1080)
    );

    // Indexes into SUPPORTED_CAMERA_RESOLUTIONS
    public static final int RESOLUTION_640x480 = 0;
    public static final int RESOLUTION_640x360 = 1;
    public static final int RESOLUTION_800x448 = 2;
    public static final int RESOLUTION_800x600 = 3;
    public static final int RESOLUTION_864x480 = 4;
    public static final int RESOLUTION_1920x1080 = 5;

    // Resolution to use for the webcams. This is an index into the SUPPORTED_CAMERA_RESOLUTIONS list.
    public static int RESOLUTION_SELECTION = RESOLUTION_864x480;

    // Positions and orientations for the webcams on this robot
    public static Position LEFT_CAMERA_POSITION = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    public static YawPitchRollAngles LEFT_CAMERA_ORIENTATION = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    public static Position RIGHT_CAMERA_POSITION = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    public static YawPitchRollAngles RIGHT_CAMERA_ORIENTATION = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private final Location location;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    /**
     * Creates a new vision sensor for the webcam with the given name.
     *
     * @param hardwareMap mapping of all the hardware on the robot
     * @param telemetry   the telemetry used to provide output on the driver station
     * @param location    the location of the webcam on the robot
     */
    public Webcam(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry, @NonNull Location location) {
        this.location = location;
        WebcamName webcamName;
        if (location == Location.LEFT) {
            webcamName = hardwareMap.get(WebcamName.class, Location.LEFT.webcamName());
            initAprilTag(webcamName, LEFT_CAMERA_POSITION, LEFT_CAMERA_ORIENTATION);
        } else {
            webcamName = hardwareMap.get(WebcamName.class, Location.RIGHT.webcamName());
            initAprilTag(webcamName, RIGHT_CAMERA_POSITION, RIGHT_CAMERA_ORIENTATION);
        }

        CameraStreamServer.getInstance().setSource(visionPortal);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag(WebcamName webcam, Position cameraPosition, YawPitchRollAngles cameraOrientation) {
        // Get the camera resolution selection
        Size cameraResolution = SUPPORTED_CAMERA_RESOLUTIONS.get(RESOLUTION_SELECTION);

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(cameraResolution)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

        // Stopping the LiveView is recommended during competition to save CPU resources when
        // a LiveView is not required for debugging purposes.
        visionPortal.stopLiveView();
    }

    /**
     * Returns indication as to whether the webcam portal used by the vision sensor is or is not
     * initialized.
     *
     * @return <code>true</code> if the vision sensor is initialized;
     *         <code>false</code> if it is not.
     */
    public boolean webcamInitialized() {
        return visionPortal != null && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    /**
     * Get a list containing the latest detections, which may be stale
     * i.e. the same as the last time you called this
     *
     * @return a list containing the latest detections.
     */
    public ArrayList<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    /**
     * Get the current rate at which frames are passing through the vision portal
     * (and all processors therein) per second - frames per second.
     *
     * @return the current vision frame rate in frames per second
     */
    public float getFps() {
        return visionPortal.getFps();
    }

    /**
     * Closes the webcam portal used by the vision sensor. The vision sensor should not be used
     * once the webcam portal has been closed.
     */
    public void close() {
        if (visionPortal != null && visionPortal.getCameraState() != VisionPortal.CameraState.CAMERA_DEVICE_CLOSED) {
            visionPortal.close();
            visionPortal = null;
        }
    }

    /**
     * Stop the streaming session. This is an asynchronous call which does not take effect
     * immediately.
     * <p>
     * Stopping the streaming session is a good way to save computational resources if there may
     * be long (e.g. 10+ second) periods of match play in which vision processing is not required.
     * When streaming is stopped, no new image data is acquired from the camera and any attached
     * {@link VisionProcessor}s will lie dormant until such time as {@link #resumeStreaming()} is called.
     * <p>
     * Stopping and starting the stream can take a second or two, and thus is not advised for use
     * cases where instantaneously enabling/disabling vision processing is required.
     */
    public void stopStreaming() {
        visionPortal.stopStreaming();
    }

    /**
     * Resume the streaming session if previously stopped by {@link #stopStreaming()}. This is
     * an asynchronous call which does not take effect immediately. If you call {@link #stopStreaming()}
     * before the operation is complete, it will SYNCHRONOUSLY await completion of the resume command.
     * <p>
     * See notes about use case on {@link #stopStreaming()}
     */
    public void resumeStreaming() {
        visionPortal.resumeStreaming();
    }

    /**
     * Gets a string representation of the webcam.
     *
     * @return a string representation of the webcam
     */
    @NonNull
    @Override
    public String toString() {
        return "Webcam{" +
                ", visionPortal=" + visionPortal +
                ", location=" + location +
                '}';
    }

    /**
     * Position of the webcam on the robot
     */
    public enum Location {
        LEFT("Webcam 1"),
        RIGHT("Webcam 2");

        private final String webcamName;

        /**
         * Instantiates a new enum with the given webcam name.
         *
         * @param webcamName the name of the webcam as configured on the REV control hub
         */
        Location(String webcamName) {
            this.webcamName = webcamName;
        }

        /**
         * Gets the name of the webcam
         *
         * @return the name of the webcam as configured on the REV control hub
         */
        public String webcamName() {
            return webcamName;
        }
    }
}
