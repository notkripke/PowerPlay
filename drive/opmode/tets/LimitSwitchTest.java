package org.firstinspires.ftc.teamcode.drive.opmode.tets;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;

@TeleOp(group = "drive")
public class LimitSwitchTest extends GorillabotsCentral {
    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();
        ElapsedTime intaketime = new ElapsedTime();

        waitForStart();

        while(!isStopRequested()){

         lift.update();
         intake.update(intaketime);
         telemetry.addData("LMT Switch: ", intake.switch_triggered);
         telemetry.addData("Limit switch: ", intake.getSwitchState());
         telemetry.update();


        }
    }
}
