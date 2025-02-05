package org.firstinspires.ftc.teamcode.roadrunner;

import static com.acmerobotics.roadrunner.ftc.OTOSKt.OTOSPoseToRRPose;
import static com.acmerobotics.roadrunner.ftc.OTOSKt.RRPoseToOTOSPose;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;

/**
 * Experimental extension of MecanumDrive that uses the SparkFun OTOS sensor for localization.
 * <p>
 * Released under the BSD 3-Clause Clear License by j5155 from 12087 Capital City Dynamics
 * Portions of this code made and released under the MIT License by SparkFun
 * Unless otherwise noted, comments are from SparkFun
 */
public class SparkFunOTOSDrive extends MecanumDrive {

    public SparkFunOTOS otos;
    private Pose2d lastOtosPose = pose;

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);

    public SparkFunOTOSDrive(HardwareMap hardwareMap, Pose2d pose) {
        super(hardwareMap, pose);
        Robot robot = Robot.getInstance();
        otos = robot.otos;
    }
    @Override
    public PoseVelocity2d updatePoseEstimate() {
        if (lastOtosPose != pose) {
            // RR localizer note:
            // Something else is modifying our pose (likely for relocalization),
            // so we override otos pose with the new pose.
            // This could potentially cause up to 1 loop worth of drift.
            // I don't like this solution at all, but it preserves compatibility.
            // The only alternative is to add getter and setters, but that breaks compat.
            // Potential alternate solution: timestamp the pose set and backtrack it based on speed?
            otos.setPosition(RRPoseToOTOSPose(pose));
        }
        // RR localizer note:
        // The values are passed by reference, so we create variables first,
        // then pass them into the function, then read from them.

        // Reading acceleration worsens loop times by 1ms,
        // but not reading it would need a custom driver and would break compatibility.
        // The same is true for speed: we could calculate speed ourselves from pose and time,
        // but it would be hard, less accurate, and would only save 1ms of loop time.
        SparkFunOTOS.Pose2D otosPose = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosVel = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D otosAcc = new SparkFunOTOS.Pose2D();
        otos.getPosVelAcc(otosPose,otosVel,otosAcc);
        pose = OTOSPoseToRRPose(otosPose);
        lastOtosPose = pose;

        // RR standard
        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));

        // RR localizer note:
        // OTOS velocity units happen to be identical to Roadrunners, so we don't need any conversion!
        return new PoseVelocity2d(new Vector2d(otosVel.x, otosVel.y),otosVel.h);
    }
}
