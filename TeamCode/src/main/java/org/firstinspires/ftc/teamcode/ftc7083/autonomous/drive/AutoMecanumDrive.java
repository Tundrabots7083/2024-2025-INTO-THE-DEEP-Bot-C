package org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.localization.Localizer;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.Drawing;
import org.firstinspires.ftc.teamcode.roadrunner.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.roadrunner.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

/**
 * A very slightly modified version of the base RoadRunner <code>MecanumDrive</code> class. This
 * particular class has four changes:
 * <ul>
 *     <li>
 *         The drive parameters, <code>Params</code>, has been moved out of the class and
 *         into a separate class, and made static constants. In addition, the IMU orientation
 *         in the <code>Params</code> class have been removed as they are not used.
 *     </li>
 *     <li>
 *         <code>MecanumDrive</code> is used to access and control the hardware instead of
 *         obtaining and setting values for the wheel motors directly within this class.
 *     </li>
 *     <li>
 *         The default localizer, which uses drive motor encoders, has been removed as it is never used.
 *         This class now uses the localizer from the <code>Robot</code> class.
 *     </li>
 *     <li>
 *         References to the IMU have been removed, as it is not used when using <code>ThreeDeadWheelLocalizer</code>.
 *     </li>
 * </ul>
 */
@Config
public class AutoMecanumDrive {
    public static int POSE_HISTORY_SIZE = 100;

    public final MecanumKinematics kinematics = new MecanumKinematics(
            Params.inPerTick * Params.trackWidthTicks, Params.inPerTick / Params.lateralInPerTick);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            Params.maxAngVel, -Params.maxAngAccel, Params.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(Params.maxWheelVel),
                    new AngularVelConstraint(Params.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(Params.minProfileAccel, Params.maxProfileAccel);

    public final MecanumDrive mecanumDrive;

    public final VoltageSensor voltageSensor;

//    public final Localizer localizer;
    protected final LinkedList<Pose2d> poseHistory = new LinkedList<>();
    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);
    public Pose2d pose;

    public AutoMecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        this.pose = pose;
        this.mecanumDrive = Robot.getInstance().mecanumDrive;

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

//        localizer = Robot.getInstance().localizer;
//        localizer.setPose2d(pose);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        double leftFrontPower = wheelVels.leftFront.get(0) / maxPowerMag;
        double leftRearPower = wheelVels.leftBack.get(0) / maxPowerMag;
        double rightRearPower = wheelVels.rightBack.get(0) / maxPowerMag;
        double rightFrontPower = wheelVels.rightFront.get(0) / maxPowerMag;

        mecanumDrive.setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);
    }

    public PoseVelocity2d updatePoseEstimate() {
//        Localizer localizer = Robot.getInstance().localizer;

//        pose = localizer.getPose2d();
        // RR standard
//        poseHistory.add(pose);
//        while (poseHistory.size() > POSE_HISTORY_SIZE) {
//            poseHistory.removeFirst();
//        }
//
//        estimatedPoseWriter.write(new PoseMessage(pose));

//        return localizer.getVelocity();
        return null;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private final double[] xPoints, yPoints;
        private double beginTs = -1;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                mecanumDrive.setMotorPowers(0, 0, 0, 0);
                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    Params.axialGain, Params.lateralGain, Params.headingGain,
                    Params.axialVelGain, Params.lateralVelGain, Params.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(Params.kS,
                    Params.kV / Params.inPerTick, Params.kA / Params.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            mecanumDrive.setMotorPowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                mecanumDrive.setMotorPowers(0, 0, 0, 0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    Params.axialGain, Params.lateralGain, Params.headingGain,
                    Params.axialVelGain, Params.lateralVelGain, Params.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(Params.kS,
                    Params.kV / Params.inPerTick, Params.kA / Params.inPerTick);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;

            mecanumDrive.setMotorPowers(leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }
}
