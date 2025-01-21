package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import android.util.Size;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Scalar;

import java.util.List;

/**
 * A webcam used to create and obtain data from a vision processor and detect the visible
 * Samples.
 */
public class GlobalShutterCamera extends SubsystemBase {


        public static final Size RESOLUTION_1920x1200 = new Size(1920,1200);


        private VisionPortal visionPortal;
        private ColorBlobLocatorProcessor colorLocator;

        /**
         * Creates a new vision sensor for the webcam with the given name.
         *
         * @param hardwareMap mapping of all the hardware on the robot
         * @param telemetry   the telemetry used to provide output on the driver station
         */
        public GlobalShutterCamera(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {

            WebcamName webcamName = hardwareMap.get(WebcamName.class, "GlobalShutterCamera");
            initWebcam(webcamName);

            CameraStreamServer.getInstance().setSource(visionPortal);
        }

        /**
         * Initialize the ColorBlobLocator processor for the webcam
         */
        private void initWebcam(WebcamName webcam) {

            // Create the ColorBlobLocator processor.
            colorLocator = new ColorBlobLocatorProcessor.Builder()
                    .setTargetColorRange(new ColorRange(ColorSpace.HSV, new Scalar( 17, 130,80), new Scalar( 29, 255,  255)))
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                    .setRoi(ImageRegion.entireFrame())  // search the entirety of camera view
                    .setDrawContours(true)                        // Show contours on the Stream Preview
                    .setBlurSize(2)                               // Smooth the transitions between different colors in image
                    .build();


            // Create the vision portal by using a builder.
            visionPortal = new VisionPortal.Builder()
                    .setCamera(webcam)
                    .setCameraResolution(RESOLUTION_1920x1200)
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .addProcessor(colorLocator)
                    .build();

            // Stopping the LiveView is recommended during competition to save CPU resources when
            // a LiveView is not required for debugging purposes.
            // visionPortal.stopLiveView();
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
        public List<ColorBlobLocatorProcessor.Blob> getDetections() {
            return colorLocator.getBlobs();
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
    }