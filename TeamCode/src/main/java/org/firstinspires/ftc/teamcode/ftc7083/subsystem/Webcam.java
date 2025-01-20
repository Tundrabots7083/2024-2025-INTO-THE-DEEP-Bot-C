package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.ftc7083.math.FTCMath;
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

    // Dimensions of the robot
    public static double BOT_LENGTH = 17.0; // 1/2 == 8.5
    public static double BOT_WIDTH = 11.375; // 1/2 == 5.6875

    // Camera position
    public static double LEFT_CAMERA_X = -5.875;
    public static double LEFT_CAMERA_Y = 7.625; // measured at 5.625
    public static double LEFT_CAMERA_Z = 5.75;
    public static double LEFT_CAMERA_YAW = 90.0;
    public static double RIGHT_CAMERA_X = 5.875;
    public static double RIGHT_CAMERA_Y = 5.5625; // measured at 4.5625
    public static double RIGHT_CAMERA_Z = 5.75;
    public static double RIGHT_CAMERA_YAW = -90.0;

    // Resolution to use for the webcams. This is an index into the SUPPORTED_CAMERA_RESOLUTIONS list.
    public static int RESOLUTION_SELECTION = RESOLUTION_800x600;

    // Position and orientation for the webcams on this robot
    public static Position LEFT_CAMERA_POSITION = new Position(DistanceUnit.INCH,
            LEFT_CAMERA_X, LEFT_CAMERA_Y, LEFT_CAMERA_Z, 0);
    public static YawPitchRollAngles LEFT_CAMERA_ORIENTATION = new YawPitchRollAngles(AngleUnit.DEGREES,
            LEFT_CAMERA_YAW, -90, 0, 0);
    public static Position RIGHT_CAMERA_POSITION = new Position(DistanceUnit.INCH,
            RIGHT_CAMERA_X, RIGHT_CAMERA_Y, RIGHT_CAMERA_Z, 0);
    public static YawPitchRollAngles RIGHT_CAMERA_ORIENTATION = new YawPitchRollAngles(AngleUnit.DEGREES,
            RIGHT_CAMERA_YAW, -90, 0, 0);

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
    public Webcam(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry, @NonNull Location location, int viewId) {
        this.location = location;

        if (location == Location.LEFT) {
            WebcamName webcamName = hardwareMap.get(WebcamName.class, location.webcamName());
            initWebcam(webcamName, LEFT_CAMERA_POSITION, LEFT_CAMERA_ORIENTATION, viewId);
        } else {
            WebcamName webcamName = hardwareMap.get(WebcamName.class, location.webcamName());
            initWebcam(webcamName, RIGHT_CAMERA_POSITION, RIGHT_CAMERA_ORIENTATION, viewId);
        }

        CameraStreamServer.getInstance().setSource(visionPortal);
    }

    /**
     * Initialize the AprilTag processor for the webcam
     */
    private void initWebcam(WebcamName webcam, Position cameraPosition, YawPitchRollAngles cameraOrientation, int containerId) {
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
                .setLiveViewContainerId(containerId)
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
     * Gets the location of the webcam.
     *
     * @return the location of the webcam
     */
    public Location getLocation() {
        return location;
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
                location +
                ", fps=" + FTCMath.round(getFps(), 2) +
                '}';
    }

    /**
     * Position of the webcam on the robot
     */
    public enum Location {
        LEFT("Left Webcam"),
        RIGHT("Right Webcam");

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
