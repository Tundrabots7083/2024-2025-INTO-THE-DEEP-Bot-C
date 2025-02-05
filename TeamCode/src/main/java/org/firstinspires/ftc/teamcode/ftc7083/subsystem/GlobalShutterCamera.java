package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import android.util.Size;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.ftc7083.localization.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.processors.ColorBlobLocatorProcessorEx;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.processors.ColorRangeEx;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.processors.ImageRegionEx;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;

import java.util.Arrays;
import java.util.List;

/**
 * A webcam used to create and obtain data from a vision processor and detect the visible
 * Samples.
 */
public class GlobalShutterCamera extends SubsystemBase {


        public static final Size RESOLUTION_1920x1200 = new Size(1920,1200);
        public static final Size RESOLUTION_1280x800 = new Size(1280,800);
        public static final Size RESOLUTION_800x600 = new Size(800,600);




        private VisionPortal visionPortal;
        private ColorBlobLocatorProcessorEx yellowColorLocator;
        private ColorBlobLocatorProcessorEx blueColorLocator;
        private ColorBlobLocatorProcessorEx redColorLocator;
        private AprilTagProcessor aprilTagProcessor;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            6.125, 6.125, 10.5, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, -90, 0);


    /**
         * Creates a new vision sensor for the webcam with the given name.
         *
         * @param hardwareMap mapping of all the hardware on the robot
         * @param telemetry   the telemetry used to provide output on the driver station
         */
        public GlobalShutterCamera(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry, GlobalShutterCameraDetectionType detectionType) {

            WebcamName webcamName;

            if(detectionType == GlobalShutterCameraDetectionType.DETECT_COLOR) {
                webcamName = hardwareMap.get(WebcamName.class, "WristGlobalShutterCamera");
            } else {
                webcamName = hardwareMap.get(WebcamName.class, "AprilTagGlobalShutterCamera");
            }

            initWebcam(webcamName, detectionType);
            CameraStreamServer.getInstance().setSource(visionPortal);
        }

        /**
         * Initialize the ColorBlobLocator processor for the webcam
         */
        private void initWebcam(WebcamName webcam, GlobalShutterCameraDetectionType detectionType) {

            switch (detectionType){
                case DETECT_COLOR:

                    // Create the ColorBlobLocator processor.
                    yellowColorLocator = new ColorBlobLocatorProcessorEx.Builder()
                            .setTargetColorRange(new ColorRangeEx(ColorSpace.HSV, new Scalar( 20, 100,80), new Scalar( 30, 255,  255)))
                            .setContourMode(ColorBlobLocatorProcessorEx.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                            .setRoi(ImageRegionEx.entireFrame())  // search the entirety of camera view
                            .setDrawContours(true)                        // Show contours on the Stream Preview
                            .setBlurSize(3)                               // Smooth the transitions between different colors in image
                            .build();

                    blueColorLocator = new ColorBlobLocatorProcessorEx.Builder()
                            .setTargetColorRange(new ColorRangeEx(ColorSpace.HSV, new Scalar( 100, 124, 53), new Scalar( 123, 255,  255)))
                            .setContourMode(ColorBlobLocatorProcessorEx.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                            .setRoi(ImageRegionEx.entireFrame())  // search the entirety of camera view
                            .setDrawContours(true)                        // Show contours on the Stream Preview
                            .setBlurSize(3)                               // Smooth the transitions between different colors in image
                            .build();


                    redColorLocator = new ColorBlobLocatorProcessorEx.Builder()
                            .setTargetColorRangeEx(new ColorRangeEx(ColorSpace.HSV, new Scalar( 135, 50, 40), new Scalar( 180, 255,  255)), new ColorRangeEx(ColorSpace.HSV, new Scalar( 0, 50, 40), new Scalar( 10, 255,  255)))
                            .setContourMode(ColorBlobLocatorProcessorEx.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                            .setRoi(ImageRegionEx.entireFrame())  // search the entirety of camera view
                            .setDrawContours(true)                        // Show contours on the Stream Preview
                            .setBlurSize(3)                               // Smooth the transitions between different colors in image
                            .build();

                    // Create the vision portal by using a builder.
                    visionPortal = new VisionPortal.Builder()
                            .setCamera(webcam)
                            .setCameraResolution(RESOLUTION_1920x1200)
                            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                            .addProcessors(yellowColorLocator, blueColorLocator, redColorLocator)
                            .setLiveViewContainerId(0)
                            .build();

                    // Stopping the LiveView is recommended during competition to save CPU resources when
                    // a LiveView is not required for debugging purposes.
                    visionPortal.stopLiveView();

                    break;
                case DETECT_APRILTAGS:
                    // Create the AprilTag processor.
                    aprilTagProcessor = new AprilTagProcessor.Builder()
                            .setCameraPose(cameraPosition, cameraOrientation)
                            .build();

                    // Create the vision portal by using a builder.
                    visionPortal = new VisionPortal.Builder()
                            .setCamera(webcam)
                            .setCameraResolution(RESOLUTION_1280x800)
                            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                            .addProcessor(aprilTagProcessor)
                            .setLiveViewContainerId(1)
                            .build();

                    // Stopping the LiveView is recommended during competition to save CPU resources when
                    // a LiveView is not required for debugging purposes.
                    visionPortal.stopLiveView();

                    break;
                default:
                    this.initWebcam(webcam, GlobalShutterCameraDetectionType.DETECT_COLOR);
            }
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
         * Get a list containing the latest yellow sample detections, which may be stale
         * i.e. the same as the last time you called this
         *
         * @return a list containing the latest detections.
         */
        public List<ColorBlobLocatorProcessorEx.Blob> getYellowDetections() {
            return yellowColorLocator.getBlobs();
        }

        /**
         * Get a list containing the latest blue sample detections, which may be stale
         * i.e. the same as the last time you called this
         *
         * @return a list containing the latest detections.
         */
        public List<ColorBlobLocatorProcessorEx.Blob> getBlueDetections() {
            return blueColorLocator.getBlobs();
        }

        /**
         * Get a list containing the latest yellow sample detections, which may be stale
         * i.e. the same as the last time you called this
         *
         * @return a list containing the latest detections.
         */
        public List<ColorBlobLocatorProcessorEx.Blob> getRedDetections() {
            return redColorLocator.getBlobs();
        }

    public List<AprilTagDetection> getAprilTagDetections() {
        return aprilTagProcessor.getDetections();
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
                    '}';
        }

    public enum GlobalShutterCameraDetectionType {
        DETECT_COLOR,
        DETECT_APRILTAGS;
    }
    }

