package org.firstinspires.ftc.teamcode.drive.opmode.tets;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Extension;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Intake;

@TeleOp(group = "drive")
@Disabled
public class ComponentTest extends GorillabotsCentral {
    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        initializeComponents();

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime intaketime = new ElapsedTime();

        double last_time = 0;


        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        drive.setPoseEstimate(new Pose2d(0,0,0));

        double feedforward_cnst = .1;

        intake.intake.setPosition(intake.intake.getPosition());

        intake.testpos = intake.intake.getPosition();

        intake.target = Intake.Position.OPEN;

        waitForStart();

        while(!isStopRequested()){

            lift.time_elapsed = timer.time() - last_time;

            if(!gamepad1.dpad_down && gamepad1.right_trigger > gamepad1.left_trigger) {
                lift.liftr.setPower(gamepad1.right_trigger + feedforward_cnst);
                lift.liftl.setPower(gamepad1.right_trigger + feedforward_cnst);
            }

            if(!gamepad1.dpad_down && gamepad1.left_trigger > gamepad1.right_trigger){
                lift.liftl.setPower(-gamepad1.left_trigger + feedforward_cnst);
                lift.liftr.setPower(-gamepad1.left_trigger + feedforward_cnst);
            }

            if(!gamepad1.dpad_down && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1){
                lift.liftr.setPower(0 + feedforward_cnst);
                lift.liftl.setPower(0 + feedforward_cnst);
            }



            /*if(!gamepad1.right_bumper && !gamepad1.left_bumper){
                extension.extension.setPower(0);
            }

            if(gamepad1.left_bumper && !gamepad1.right_bumper){
                extension.extension.setPower(-1);
            }
            if(gamepad1.right_bumper && !gamepad1.left_bumper){
                extension.extension.setPower(1);
            }*/

            if(gamepad1.dpad_down && gamepad1.right_trigger > 0.1 && gamepad1.left_trigger < 0.1){
                extension.extension.setPower(gamepad1.right_trigger);
            }
            if(gamepad1.dpad_down && gamepad1.right_trigger < 0.1 && gamepad1.left_trigger > 0.1){
                extension.extension.setPower(-gamepad1.left_trigger);
            }
            if(gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1){
                extension.extension.setPower(0);
            }


            if(gamepad1.a){
                intake.target = Intake.Position.OPEN;
            }
            if(gamepad1.b){
                intake.target = Intake.Position.CLOSED;
            }

            intake.intake.setPosition(intake.target_pos);
            intake.update(intaketime);


            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y*.7,
                            -gamepad1.left_stick_x*.7,
                            -gamepad1.right_stick_x*.7
                    )
            );

            drive.update();
            extension.update(lift.time_elapsed);
            extension.extension.setPower(extension.out);


            /*if(gamepad1.a){
                intake.intake.setPosition(.05);
            }
            if(gamepad1.b){
                intake.intake.setPosition(.95);
            }*/

            last_time = timer.time();

            dashboardTelemetry.addData("Lift L: ", lift.getPositionL());
            dashboardTelemetry.addData("Lift R: ", lift.getPositionR());
            dashboardTelemetry.addData("Intake Target: ", intake.target);
            dashboardTelemetry.addData("Intake State: ", intake.state);
            dashboardTelemetry.addData("Intake Pos: ", intake.intake.getPosition());
            dashboardTelemetry.addData("Extention Pos: ", extension.getPosition());
            dashboardTelemetry.addData("Switch: ", intake.getSwitchState());
            dashboardTelemetry.update();



        }

    }
}
