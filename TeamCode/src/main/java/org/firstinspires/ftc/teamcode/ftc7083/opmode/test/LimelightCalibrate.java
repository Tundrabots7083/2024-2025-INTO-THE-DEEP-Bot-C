package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Limelight;


@TeleOp(name = "Limelight Config", group = "test")
public class LimelightCalibrate  extends OpMode {

    private Limelight limelight;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        limelight = new Limelight(hardwareMap,telemetry);

        telemetry.setMsTransmissionInterval(11);
        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {

        Double xDistance = limelight.getDistance(Limelight.TargetPosition.SUBMERSIBLE);

        telemetry.addData("Distance to Target:",xDistance);
        telemetry.update();
    }


}
