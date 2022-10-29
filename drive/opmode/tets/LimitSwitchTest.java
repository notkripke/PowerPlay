/*package org.firstinspires.ftc.teamcode.drive.opmode.tets;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;

@TeleOp(group = "drive")
public class LimitSwitchTest extends GorillabotsCentral {
    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        waitForStart();

        while(!isStopRequested()){

         lift.update();
         telemetry.addData("Limit switch: ", intake.getSwitchState());
         telemetry.update();


        }
    }
}
*/