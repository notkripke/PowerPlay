package org.firstinspires.ftc.teamcode.drive.opmode.tets;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Lift;

@TeleOp(group = "drive")
@Disabled
public class ExtensionTest extends GorillabotsCentral {
    @Override
    public void runOpMode() throws InterruptedException {


        initializeComponents();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        ElapsedTime timer = new ElapsedTime();

        double last_time = 0;

        waitForStart();

        while(!isStopRequested()){

            //lift.setTarget(Lift.lift_stack);

            lift.time_elapsed = timer.time() - last_time;
            //lift.update();
            extension.update(lift.time_elapsed);

            //lift.liftr.setPower(lift.outR);
            //lift.liftl.setPower(lift.outL);
            extension.extension.setPower(extension.out);

            last_time = timer.time();

            dashboardTelemetry.addData("Lift L: ", lift.getPositionL());
            dashboardTelemetry.addData("Lift R: ", lift.getPositionR());
            dashboardTelemetry.addData("Extention Pos: ", extension.getPosition());
            dashboardTelemetry.update();
        }
    }
}
