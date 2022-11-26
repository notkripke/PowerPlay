package org.firstinspires.ftc.teamcode.drive.opmode.tets;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;

@TeleOp(group = "drive")
public class DistanceTest extends GorillabotsCentral {
    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        int lp = 0;

        waitForStart();

        while(!isStopRequested()){

            lp += 1;

            if(lp > 30){ lp = 0; }


            sensors.update(true, 30, lp);
            dashboardTelemetry.addData("Intake Distance: ", sensors.dIn.getDistance(DistanceUnit.INCH));
            dashboardTelemetry.update();
        }



    }
}
