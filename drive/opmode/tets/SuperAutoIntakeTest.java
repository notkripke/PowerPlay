package org.firstinspires.ftc.teamcode.drive.opmode.tets;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;

public class SuperAutoIntakeTest extends GorillabotsCentral {
    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        ElapsedTime sensorTimer = new ElapsedTime();

        double sensorTimeGap = 150;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();

        while(!isStarted()){


            sensors.updateAutoAdjTimer(sensorTimer, sensorTimeGap, false, true);
            drive.setWeightedDrivePower(new Pose2d(sensors.adj_power_x, sensors.adj_power_y, 0));
            drive.update();

            dashboardTelemetry.addData("in_dist: ", sensors.in_dist);
            dashboardTelemetry.addData("in_side_dist: ", sensors.in_side_dist);
            dashboardTelemetry.addData("adj_x: ", sensors.adj_power_x);
            dashboardTelemetry.addData("adj_y: ", sensors.adj_power_y);
            dashboardTelemetry.update();
        }
    }
}
