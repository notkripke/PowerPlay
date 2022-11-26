package org.firstinspires.ftc.teamcode.drive.opmode.tets;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Extension;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Intake;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Lift;

@TeleOp(group = "drive")
public class AutoThingsTest extends GorillabotsCentral {
    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        initializeComponents();

        ElapsedTime timer = new ElapsedTime();

        ElapsedTime intaketime = new ElapsedTime();

        double last_time = 0;

        boolean act = true;

        int snsr_loop = 0;

        int loop_max = 40;

        lift.setTarget(lift.lift_hold);

        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        drive.setPoseEstimate(new Pose2d(0,0,0));

        intake.target = Intake.Position.OPEN;
        intake.last_target = Intake.Position.OPEN;

        double feedforward_cnst = .2;

        waitForStart();

        lift.setTarget(lift.lift_hold);

        intake.target = Intake.Position.OPEN;
        intake.last_target = Intake.Position.OPEN;

        while(!isStopRequested()){

            snsr_loop += 1;
            if(snsr_loop > loop_max){
                snsr_loop = 0;
            }

            lift.time_elapsed = timer.time() - last_time;

            if(gamepad1.right_bumper && !gamepad1.left_bumper){
                extension.setTarget(Extension.full_ext);
            }
            if(gamepad1.left_bumper && !gamepad1.right_bumper){
                extension.setTarget(Extension.intake_pos);
            }
            // if(!gamepad1.right_bumper && !gamepad1.left_bumper){
            //   extension.extension.setPower(0);
            //}

            if(gamepad1.a){
                intake.target = Intake.Position.OPEN;
            }
            if(gamepad1.b){
                intake.target = Intake.Position.CLOSED;
            }

            if(lift.target == lift.lift_hold && (lift.posL > lift.target * 0.8)){
                if(sensors.intakeReady){
                    lift.setTarget(1);
                }
            }

            if(intake.state == Intake.State.CLOSED && intake.switch_cooldown && lift.target == 1){
                lift.setTarget(Lift.lift_stack);
            }

            if(intake.state == Intake.State.CLOSED && lift.posL > lift.lift_stack * .8){
                if(gamepad1.y){ lift.setTarget(Lift.lift_low); }
            }

            if(lift.last_posL > lift.lift_low * 0.8 && intake.state == Intake.State.CLOSED){
                extension.setTarget(Extension.full_ext);
            }

            if(lift.last_posL > lift.lift_low * .9 && intake.state == Intake.State.OPEN){
                extension.setTarget(Extension.intake_pos);
            }

            if(lift.posL > lift.lift_low * .9 && extension.pos < 300 && intake.state == Intake.State.OPEN){
                lift.setTarget(Lift.lift_hold);
            }

            /*
            if(intake.getSwitchState()){
                //intake.intake.setPosition(intake.CLOSED);
                intake.target = Intake.Position.CLOSED;
            }
            if(!intake.getSwitchState()){
                //intake.intake.setPosition(intake.OPEN);
                intake.target = Intake.Position.OPEN;
            }*/

            intake.update(intaketime);
            intake.intake.setPosition(intake.target_pos);

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
            sensors.update(act, snsr_loop, loop_max);
            lift.updateFeedforwardNew();
            lift.liftl.setPower(lift.outL);
            lift.liftr.setPower(lift.outL);


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
            dashboardTelemetry.addData("Distance: ", sensors.in_dist);
            dashboardTelemetry.addData("Within Intake Range?: ", sensors.intakeReady);
            dashboardTelemetry.addData("Lift Target: ", lift.target);
            dashboardTelemetry.update();



        }

    }
}
