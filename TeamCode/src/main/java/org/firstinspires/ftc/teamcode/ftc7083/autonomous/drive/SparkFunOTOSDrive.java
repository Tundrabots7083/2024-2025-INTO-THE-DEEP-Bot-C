package org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive;

import static com.acmerobotics.roadrunner.ftc.OTOSKt.OTOSPoseToRRPose;
import static com.acmerobotics.roadrunner.ftc.OTOSKt.RRPoseToOTOSPose;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.localization.Localizer;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;

/**
 * Experimental extension of MecanumDrive that uses the SparkFun OTOS sensor for localization.
 * <p>
 * Released under the BSD 3-Clause Clear License by j5155 from 12087 Capital City Dynamics
 * Portions of this code made and released under the MIT License by SparkFun
 * Unless otherwise noted, comments are from SparkFun
 */
public class SparkFunOTOSDrive extends AutoMecanumDrive {
    private static final int POSE_HISTORY_SIZE = 100;

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private Pose2d lastOtosPose = pose;
    public final SparkFunOTOS otos;

    public SparkFunOTOSDrive(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);

        Robot robot = Robot.getInstance();
        otos = robot.otos;
        otos.setPosition(RRPoseToOTOSPose(pose));
    }

    @SuppressLint("DefaultLocale")
    @Override
    public PoseVelocity2d updatePoseEstimate() {
        Robot robot = Robot.getInstance();
        Telemetry telemetry = robot.telemetry;
        Localizer localizer = robot.localizer;

        if (lastOtosPose != pose) {
            // RR localizer note:
            // Something else is modifying our pose (likely for relocalization),
            // so we override otos pose with the new pose.
            // This could potentially cause up to 1 loop worth of drift.
            // I don't like this solution at all, but it preserves compatibility.
            // The only alternative is to add getter and setters, but that breaks compat.
            // Potential alternate solution: timestamp the pose set and backtrack it based on speed?
            localizer.setPose2d(pose);
        }
        pose = localizer.getPose2d();
        lastOtosPose = pose;

        // RR standard
        poseHistory.add(pose);
        while (poseHistory.size() > POSE_HISTORY_SIZE) {
            poseHistory.removeFirst();
        }
        estimatedPoseWriter.write(new PoseMessage(pose));
        
        PoseVelocity2d vel = localizer.getVelocity();

        telemetry.addLine(String.format("[Auto] pose: x=%.2f, y=%.2f, h=%.2f", pose.position.x, pose.position.y, pose.heading.toDouble()));
        telemetry.addLine(String.format("[Auto] vel: x=%.2f, y=%.2f, ang=%.2f", vel.linearVel.x, vel.linearVel.y,  vel.angVel));

        return vel;
    }
}
