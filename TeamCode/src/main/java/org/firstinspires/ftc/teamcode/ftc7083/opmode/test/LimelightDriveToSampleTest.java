package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.math.FTCMath;

/**
 * A test OpMode that drives the robot to a detected sample on the field. The color of the
 * sample to drive to is set based on <code>Robot.INTAKE_COLOR</code>. Tuning constants are in
 * <code>Robot.DriveTo</code>.
 */
@TeleOp(name = "Limelight Drive To Sample", group = "tests")
public class LimelightDriveToSampleTest extends OpMode {
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private Robot robot;
    private Action driveToSample;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        robot = Robot.init(hardwareMap, telemetry);
        driveToSample = robot.mecanumDrive.actionDriveToSample(robot.limelight);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        switch (Robot.INTAKE_COLOR) {
            case RED:
                robot.limelight.detectRed();
                break;
            case BLUE:
                robot.limelight.detectBlue();
                break;
            default:
                robot.limelight.detectYellow();
        }
    }

    @Override
    public void loop() {
        timer.reset();

        // Clear the bulk cache for each Lynx module hub. This must be performed once per loop
        // as the bulk read caches are being handled manually.
        for (LynxModule hub : robot.allHubs) {
            hub.clearBulkCache();
        }

        // Run the RoadRunner action. Normally, this is only called until the method returns
        // 'false', but for this sample OpMode we continuously run it.
        driveToSample.run(new TelemetryPacket());

        // Print out the loop time, in milliseconds, to two decimal places
        telemetry.addData("Loop Time", FTCMath.round(timer.time(), 2));
        telemetry.update();
    }
}
