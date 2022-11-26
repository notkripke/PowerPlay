package org.firstinspires.ftc.teamcode.drive.opmode.tets;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.ExtentionMAG;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Intake;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Lift;


@TeleOp(group = "drive")
public class MagAutoTeleop extends GorillabotsCentral {

    enum FSM{
        INTAKE_UP,
        INTAKE_DOWN,
        TRANSFER,
        RAISE,
        RETURN
    }

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        initializeComponents();

        ExtentionMAG extentionMAG = new ExtentionMAG(hardwareMap);

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime exttimer = new ElapsedTime();
        ElapsedTime intaketime = new ElapsedTime();

        FSM last_mchn = FSM.INTAKE_UP;

        boolean reintake_watch = false;

        double drive_speed_constant = .7;

        double last_time = 0;

        boolean act = false;

        boolean off_stack = false;

        double custom_transfer_target = 0;

        int snsr_loop = 0;

        int loop_max = 40;

        lift.setTarget(lift.lift_hold);

        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        drive.setPoseEstimate(new Pose2d(0,0,0));

        intake.target = Intake.Position.OPEN;
        intake.last_target = Intake.Position.OPEN;

        double feedforward_cnst = .2;

        FSM machine = FSM.INTAKE_UP;

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


            switch(machine){

                case INTAKE_UP:

                    lift.setTarget(Lift.lift_stack);
                    intake.target = Intake.Position.OPEN;
                    //extension.setTarget(Extension.intake_pos);
                    extentionMAG.setTarget(ExtentionMAG.State.RETRACTED);
                    drive_speed_constant = 0.6;




                    if(lift.state == Lift.State.HOLDING && intake.state == Intake.State.OPEN && lift.target == lift.lift_stack){

                        act = true;

                        if(sensors.intakeReady){
                            lift.setTarget(1);
                            machine = FSM.INTAKE_DOWN;
                        }
                    }

                    break;

                case INTAKE_DOWN:

                    act = false;

                    if(intake.state == Intake.State.CLOSED && intake.switch_cooldown){

                        if(lift.state == Lift.State.STALLING && lift.last_posL > 75){
                            off_stack = true;
                            custom_transfer_target = lift.last_posL + 150;
                        }

                        if((lift.state == Lift.State.HOLDING || lift.state == Lift.State.STALLING) && lift.last_posL < 75){
                            off_stack = false;
                        }
                        machine = FSM.TRANSFER;
                    }

                    if((gamepad1.y && !intake.switch_triggered) || (lift.state == Lift.State.STALLING && !intake.switch_triggered)){
                        lift.setTarget(Lift.lift_stack);
                        machine = FSM.INTAKE_UP;
                    }

                    break;


                case TRANSFER:

                    if(!off_stack) {
                        lift.setTarget(lift.lift_hold);
                    }

                    if(off_stack){
                        lift.setTarget(custom_transfer_target);
                    }

                    drive_speed_constant = 0.8;

                    if(gamepad2.y){
                        lift.setTarget(Lift.lift_high);
                        machine = FSM.RAISE;
                    }
                    if(gamepad2.x){
                        lift.setTarget(Lift.lift_mid);
                        machine = FSM.RAISE;
                    }

                    if(gamepad2.b){
                        lift.setTarget(lift.lift_low);
                        machine = FSM.RAISE;
                    }

                    break;


                case RAISE:

                    drive_speed_constant = 0.4;

                    if(lift.safeToExtend){
                        //extension.setTarget(Extension.full_ext);
                        extentionMAG.setTarget(ExtentionMAG.State.EXTENDED);
                    }

                    if(lift.state == Lift.State.HOLDING && extentionMAG.state == ExtentionMAG.State.EXTENDED && intake.state == Intake.State.OPEN){
                        //intake.override = true;
                        //intake.target = Intake.Position.OPEN;
                        machine = FSM.RETURN;
                    }

                    break;

                case RETURN:

                    extentionMAG.updateSafeToLower(exttimer);
                    if(intake.state == Intake.State.OPEN && gamepad2.dpad_down){
                        //extension.setTarget(Extension.intake_pos);
                        extentionMAG.setTarget(ExtentionMAG.State.RETRACTED);
                    }
                   /* if(extension.state == Extension.State.MOVING && extension.safeToLower){
                        lift.setTarget(Lift.lift_hold);
                        machine = FSM.INTAKE_UP;
                    }*/

                    if ((extentionMAG.state == ExtentionMAG.State.MOVING_BACK && !extentionMAG.safeToLower) || extentionMAG.state == ExtentionMAG.State.RETRACTED) {

                        lift.setTarget(Lift.lift_hold);
                        machine = FSM.INTAKE_UP;
                    }

                    break;


            }

            if(gamepad2.a){
                intake.target = Intake.Position.OPEN;
            }
            if(gamepad2.back){
                intake.target = Intake.Position.CLOSED;
            }

            intake.update(intaketime);
            intake.intake.setPosition(intake.target_pos);

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * drive_speed_constant,
                            -gamepad1.left_stick_x * drive_speed_constant,
                            -gamepad1.right_stick_x * drive_speed_constant
                    )
            );

            drive.update();
            //extension.update(lift.time_elapsed);
            //extension.extension.setPower(extension.out);
            extentionMAG.update(exttimer);
            extentionMAG.extension.setPower(extentionMAG.out);
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
            dashboardTelemetry.addData("Switch: ", intake.getSwitchState());
            dashboardTelemetry.addData("Distance: ", sensors.in_dist);
            dashboardTelemetry.addData("Within Intake Range?: ", sensors.intakeReady);
            dashboardTelemetry.addData("Lift Target: ", lift.target);
            dashboardTelemetry.addData("Lift state: ", lift.state);
            dashboardTelemetry.addData("Machine: ", machine);
            dashboardTelemetry.addData("Ext State: ", extentionMAG.state);
            dashboardTelemetry.addData("Ext target: ", extentionMAG.target);
            dashboardTelemetry.addData("safeToLower: ", extentionMAG.safeToLower);
            dashboardTelemetry.update();

            last_mchn = machine;

        }

    }
}
