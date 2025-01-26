package org.firstinspires.ftc.teamcode.ftc7083.localization;

import static com.acmerobotics.roadrunner.ftc.OTOSKt.OTOSPoseToRRPose;
import static com.acmerobotics.roadrunner.ftc.OTOSKt.RRPoseToOTOSPose;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;

/**
 * A localizer that uses the SparkFun OTOS to determine the field pose and velocity.
 */
public class SparkFunOTOSLocalizer implements Localizer {
    private final SparkFunOTOS otos;
    private Pose2d pose;
    private SparkFunOTOS.Pose2D otosVel;

    /**
     * Instantiates a new SparkFun OTOS localizer, using the SparkFun OTOS provided to calculate
     * the field position.
     *
     * @param otos the SparkFun OTOS to use for localization
     */
    public SparkFunOTOSLocalizer(SparkFunOTOS otos) {
        this.otos = otos;
    }

    @Override
    public void update() {
        com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D otosPose = new com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D();
        otosVel = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosAcc = new SparkFunOTOS.Pose2D();
        otos.getPosVelAcc(otosPose, otosVel, otosAcc);
        pose = OTOSPoseToRRPose(otosPose);
    }

    @Override
    public Pose2d getPose2d() {
        return pose;
    }

    @Override
    public void setPose2d(Pose2d pose) {
        otos.setPosition(RRPoseToOTOSPose(pose));
        Telemetry telemetry = Robot.getInstance().telemetry;
        telemetry.addData("[OTOS] re-localize", pose);
    }

    @Override
    public PoseVelocity2d getVelocity() {
        return new PoseVelocity2d(new Vector2d(otosVel.x, otosVel.y), otosVel.h);
    }
}
