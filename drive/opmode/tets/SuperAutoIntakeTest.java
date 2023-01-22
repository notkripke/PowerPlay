package org.firstinspires.ftc.teamcode.drive.opmode.tets;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;

@Disabled
@TeleOp(group = "AAAAAAAAAA", name = "thingtest")
public class SuperAutoIntakeTest extends GorillabotsCentral {
    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        ElapsedTime sensorTimer = new ElapsedTime();

        int snsr_loop = 0;

        int loop_max = 40;

        double sensorTimeGap = 10;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        while (!isStarted() && !isStopRequested()) {
            sensors.updateAutoAdjTimer(sensorTimer, sensorTimeGap, false, true);
            dashboardTelemetry.addData("in_dist: ", sensors.in_dist);
            dashboardTelemetry.addData("in_side_dist: ", sensors.in_side_dist);
            dashboardTelemetry.addData("out dist: ", sensors.getDistOut());
            dashboardTelemetry.addData("adj_x: ", sensors.adj_power_x);
            dashboardTelemetry.addData("adj_y: ", sensors.adj_power_y);
            dashboardTelemetry.update();
        }

        waitForStart();

        while(opModeIsActive()){

            snsr_loop += 1;
            if (snsr_loop > loop_max) {
                snsr_loop = 0;
            }



            sensors.updateb(true, snsr_loop, loop_max/*, false, true*/);
            dashboardTelemetry.addData("in_dist: ", sensors.in_dist);
            dashboardTelemetry.addData("in_side_dist: ", sensors.in_side_dist);
            dashboardTelemetry.addData("adj_x: ", sensors.adj_power_x);
            dashboardTelemetry.addData("adj_y: ", sensors.adj_power_y);
            dashboardTelemetry.addData("intakeable?: ", sensors.intakeReady);
            dashboardTelemetry.addData("out dist: ", sensors.getDistOut());
            dashboardTelemetry.update();
        }
    }
}
