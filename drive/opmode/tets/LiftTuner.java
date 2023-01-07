package org.firstinspires.ftc.teamcode.drive.opmode.tets;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Lift;


@Disabled
@TeleOp(group = "drive")
@Config
public class LiftTuner extends GorillabotsCentral {

    static String test_mode = "PID";
    static boolean left = true;
    static boolean right = false;
    static double height = -1000;


    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime other_timer = new ElapsedTime();

        Lift.State last_state = Lift.State.BASE;

        boolean high_position = false;

        double last_time = 0;

        initializeComponents();

        lift.resetEncoders();

        lift.liftl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.liftr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        lift.init_pos = lift.liftl.getCurrentPosition();

        while(!isStopRequested()){

        lift.time_elapsed = timer.milliseconds() - last_time;

        switch (test_mode) {

            case "manual":

                if (left && !right) {
                    if (gamepad1.right_trigger > 0.15 && gamepad1.left_trigger < 0.15) {
                        lift.liftl.setPower(gamepad1.right_trigger);
                    }

                    if (gamepad1.right_trigger < 0.15 && gamepad1.left_trigger > 0.15) {
                        lift.liftl.setPower(-gamepad1.left_trigger);
                    }

                    if (gamepad1.left_trigger < 0.15 && gamepad1.right_trigger < 0.15) {
                        lift.liftl.setPower(0);
                    }
                }

                if (!left && right) {
                    if (gamepad1.right_trigger > 0.15 && gamepad1.left_trigger < 0.15) {
                        lift.liftr.setPower(gamepad1.right_trigger);
                    }

                    if (gamepad1.right_trigger < 0.15 && gamepad1.left_trigger > 0.15) {
                        lift.liftr.setPower(-gamepad1.left_trigger);
                    }

                    if (gamepad1.left_trigger < 0.15 && gamepad1.right_trigger < 0.15) {
                        lift.liftr.setPower(0);
                    }
                }

                if (left && right) {
                    if (gamepad1.right_trigger > 0.15 && gamepad1.left_trigger < 0.15) {
                        lift.liftr.setPower(gamepad1.right_trigger);
                        lift.liftl.setPower(gamepad1.right_trigger);
                    }

                    if (gamepad1.right_trigger < 0.15 && gamepad1.left_trigger > 0.15) {
                        lift.liftr.setPower(-gamepad1.left_trigger);
                        lift.liftl.setPower(-gamepad1.left_trigger);
                    }

                    if (gamepad1.left_trigger < 0.15 && gamepad1.right_trigger < 0.15) {
                        lift.liftr.setPower(0);
                        lift.liftl.setPower(0);
                    }
                }

                dashboardTelemetry.addData("Left Position: ", lift.getPositionL());
                dashboardTelemetry.addData("Right Position: ", lift.getPositionR());
                dashboardTelemetry.update();
                break;

            case "PID":

               /* if(high_position){
                    lift.target = height;
                }

                if(high_position){
                    lift.target = 0;
                }

                if(last_state != lift.state){
                    other_timer.reset();
                }

                if(other_timer.seconds() > 5){
                    high_position =! high_position;
                }*/

                lift.updateFeedforwardNew();




                last_state = lift.state;

                lift.liftr.setPower(lift.outL);
                lift.liftl.setPower(lift.outL);

                dashboardTelemetry.addData("Target Position: ", lift.target);
                dashboardTelemetry.addData("Left Position: ", lift.posL);
                dashboardTelemetry.addData("Right Position: ", lift.posR);
                dashboardTelemetry.addData("Left Power: ", lift.outL);
                dashboardTelemetry.addData("Right Power: ", lift.outL);
                dashboardTelemetry.addData("Lift State: ", lift.state);
                dashboardTelemetry.addData("Int Sum L: ", lift.integral_sumL);
                dashboardTelemetry.addData("Int Sum R: ", lift.integral_sumR);
                dashboardTelemetry.update();

                break;


        }

        last_time = timer.milliseconds();
        lift.time_overall = timer.milliseconds();
        }



    }
}
