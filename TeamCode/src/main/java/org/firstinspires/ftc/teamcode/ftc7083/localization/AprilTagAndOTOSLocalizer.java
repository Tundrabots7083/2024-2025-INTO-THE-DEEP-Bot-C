package org.firstinspires.ftc.teamcode.ftc7083.localization;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;

import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Webcam;

import java.util.List;

/**
 * A localizer that uses the April Tag localizer and the SparkFunOTOS localizer. If an April Tag
 * is detected by the webcams, then the localization information from the April Tag localizer is
 * used, and the current SparkFun OTOS pose is updated to match the value calculated by the
 * April Tag localizer. If no April Tags are detected, the information from the SparkFun OTOS is
 * used.
 * <p>
 * This allows April Tags to be used for localization whenever they can be detected by the robot,
 * and the SparkFun OTOS localizer used whenever April Tags cannot be detected.
 */
public class AprilTagAndOTOSLocalizer implements Localizer {
    private final AprilTagLocalizer aprilTagLocalizer;
    private final SparkFunOTOSLocalizer otosLocalizer;

    private Pose2d pose;
    private PoseVelocity2d velocity;

    /**
     * Instantiates a new localizer that uses the list of webcams and the SparkFun OTOS to
     * determine the localization information for the robot.
     *
     * @param webcams thw list of webcams used for April Tag localization
     * @param otos    the SparkFun OTOS used for localization
     */
    public AprilTagAndOTOSLocalizer(List<Webcam> webcams, SparkFunOTOSCorrected otos) {
        aprilTagLocalizer = new AprilTagLocalizer(webcams);
        otosLocalizer = new SparkFunOTOSLocalizer(otos);
    }

    @Override
    public void update() {
        aprilTagLocalizer.update();
        otosLocalizer.update();

        pose = otosLocalizer.getPose2d();
        velocity = otosLocalizer.getVelocity();

        if (aprilTagLocalizer.aprilTagsDetected()) {
            otosLocalizer.setPose2d(aprilTagLocalizer.getPose2d());
        }
    }

    @Override
    public Pose2d getPose2d() {
        return pose;
    }

    @Override
    public void setPose2d(Pose2d pose) {
        aprilTagLocalizer.setPose2d(pose);
        otosLocalizer.setPose2d(pose);
    }

    @Override
    public PoseVelocity2d getVelocity() {
        return velocity;
    }
}
