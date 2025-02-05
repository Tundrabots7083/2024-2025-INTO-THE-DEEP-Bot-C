package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.GlobalShutterCamera;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.processors.SampleProcessor;

@TeleOp(name = "GlobalShutterCamTest", group = "Concept")
public class GlobalShutterSeparatedTest extends LinearOpMode {

        @Override
        public void runOpMode()
        {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

            GlobalShutterCamera globalShutterCamera = new GlobalShutterCamera(hardwareMap, telemetry, GlobalShutterCamera.GlobalShutterCameraDetectionType.DETECT_COLOR);
            SampleProcessor sampleProcessor = new SampleProcessor();


            // WARNING:  To be able to view the stream preview on the Driver Station, this code runs in INIT mode.
            while (opModeIsActive() || opModeInInit())
            {
                telemetry.addData("preview on/off", "... Camera Stream\n");


                    Double yellowAngle = sampleProcessor.execute(globalShutterCamera.getYellowDetections());
                    // double blueAngle = sampleProcessor.execute(wristGlobalShutterCamera.getBlueDetections());
                    // double redAngle = sampleProcessor.execute(wristGlobalShutterCamera.getRedDetections());

                    telemetry.addData("Yellow Angle", yellowAngle);
                    // telemetry.addData("Blue Angle", blueAngle);
                    // telemetry.addData("Red Angle", redAngle);
                    telemetry.update();
                sleep(50);
            }
        }
}
