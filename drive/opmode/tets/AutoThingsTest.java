package org.firstinspires.ftc.teamcode.drive.opmode.tets;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

@TeleOp(group = "drive")
public class AutoThingsTest extends GorillabotsCentral {
    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        initializeComponents();

        ElapsedTime timer = new ElapsedTime();

        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        waitForStart();

        while (!isStopRequested()) {

            if(!sensors.intakeReady && gamepad1.a){
                sensors.autoStackAdjust(timer, 0.75);
                drive.setWeightedDrivePower(new Pose2d(sensors.adj_power_x, sensors.adj_power_y, 0));
            }

        }
    }
}