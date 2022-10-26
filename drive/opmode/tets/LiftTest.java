package org.firstinspires.ftc.teamcode.drive.opmode.tets;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;


@Autonomous(group = "drive")
public class LiftTest extends GorillabotsCentral {
    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        boolean sync = false;

        double buffer = 0.5;

        double time_storage = 0;

        waitForStart();

        while(!isStopRequested()){

            if(sync){

                if(gamepad1.right_trigger > gamepad1.left_trigger) {
                    lift.liftr.setPower(gamepad1.right_trigger);
                    lift.liftl.setPower(gamepad1.right_trigger);
                }

                if(gamepad1.left_trigger > gamepad1.right_trigger){
                    lift.liftl.setPower(-gamepad1.left_trigger);
                    lift.liftr.setPower(-gamepad1.left_trigger);
                }
            }

            if(!sync){
                if((gamepad1.right_trigger > gamepad1.left_trigger) && gamepad1.right_trigger > 0.2) {
                    lift.liftr.setPower(gamepad1.right_trigger);
                }

                if((gamepad1.left_trigger > gamepad1.right_trigger) && gamepad1.left_trigger > 0.2){
                    lift.liftr.setPower(-gamepad1.left_trigger);
                }
            }

            if(gamepad1.right_trigger < 0.2 && gamepad1.left_trigger < 0.2){
                lift.liftl.setPower(0);
                lift.liftr.setPower(0);
            }

            if((timer.seconds() - time_storage > buffer) && gamepad1.a){
                sync =! sync;
                time_storage = timer.seconds();
            }

            telemetry.addData("Synced?: ", sync);
            telemetry.addData("Position L: ", lift.getPositionL());
            telemetry.addData("Position R: ", lift.getPositionR());
            telemetry.update();
        }


    }
}
