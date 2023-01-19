package org.firstinspires.ftc.teamcode.drive.opmode.tets;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.ExtentionMAG;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Intake;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Lift;


@TeleOp(group = "drive")
public class DoubleReallyNewCorningTeleop extends GorillabotsCentral {

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
        ElapsedTime manualinttimer = new ElapsedTime();
        ElapsedTime stalltimer = new ElapsedTime();
        ElapsedTime liftautoadjusttimer = new ElapsedTime();
        ElapsedTime dropconetimer = new ElapsedTime();
        ElapsedTime intakedroptimer = new ElapsedTime();
        ElapsedTime manualintaketimer = new ElapsedTime();

        FSM last_mchn = FSM.INTAKE_UP;

        double lift_raise_target = 0;

        boolean override_lift_update = false;

        boolean last_intake_target_open = true;

        boolean reintake_watch = false;

        double drive_speed_constant = .7;

        double last_time = 0;

        boolean lift_high_stall_check_override = false;

        boolean act = false;

        boolean off_stack = false;

        double custom_transfer_target = 0;

        int snsr_loop = 0;

        int loop_max = 40;

        String manual_lift_stage = "";

        boolean manual_intake = false;

        boolean hasReachedTarget = false;

        boolean isAutoControlled = true;

        double controller_trigger_r = 0;

        double controller_trigger_l = 0;

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

        while(!isStopRequested()) {

            if (isAutoControlled) {

                snsr_loop += 1;
                if (snsr_loop > loop_max) {
                    snsr_loop = 0;
                }

                lift.time_elapsed = timer.time() - last_time;


                switch (machine) {

                    case INTAKE_UP:

                        override_lift_update = false;

                        if(!override_lift_update && extentionMAG.state == ExtentionMAG.State.RETRACTED) {
                            lift.setTarget(Lift.lift_stack);
                        }

                        if(gamepad2.dpad_right){
                            override_lift_update = true;
                        }
                        if(gamepad2.dpad_left){
                            override_lift_update = false;
                        }

                        if(override_lift_update){

                            if (gamepad2.right_trigger < 0.15 && gamepad2.left_trigger < 0.15) {
                                lift.liftr.setPower(lift.Rf);
                                lift.liftl.setPower(lift.Lf);
                                manual_lift_stage = "1";
                            }

                            if (gamepad2.right_trigger > 0.15 && gamepad2.left_trigger < 0.15) {
                                lift.liftl.setPower(gamepad2.right_trigger * 0.65 + lift.Lf);
                                lift.liftr.setPower(gamepad2.right_trigger * 0.65 + lift.Rf);
                                manual_lift_stage = "2";
                            }

                            if (gamepad2.left_trigger > 0.15 && gamepad2.right_trigger < 0.15) {
                                lift.liftr.setPower(-gamepad2.left_trigger * 0.4 + lift.Rf);
                                lift.liftl.setPower(-gamepad2.left_trigger * 0.4 + lift.Lf);
                                manual_lift_stage = "3";
                            }

                            if (gamepad2.right_trigger > 0.15 && gamepad2.left_trigger > 0.15) {
                                lift.liftr.setPower(lift.Rf);
                                lift.liftl.setPower(lift.Lf);
                                manual_lift_stage = "4";
                            }


                        }


                        intake.target = Intake.Position.OPEN;
                        last_intake_target_open = true;
                        //extension.setTarget(Extension.intake_pos);
                        extentionMAG.setTarget(ExtentionMAG.State.RETRACTED);
                        drive_speed_constant = 0.6;


                        if (lift.state == Lift.State.HOLDING && intake.state == Intake.State.OPEN && lift.target == lift.lift_stack && lift.last_posL > 500) {

                            act = true;


                            if (sensors.intakeReady) {
                                lift.setTarget(1);
                                machine = FSM.INTAKE_DOWN;
                            }
                        }

                        break;

                    case INTAKE_DOWN:

                        if(gamepad1.b){
                            intake.intake.setPosition(intake.CLOSED);
                            intake.target = Intake.Position.CLOSED;
                        }

                        lift.max_power_dwn = -0.35;

                        act = false;

                        if(!intake.switch_triggered){
                            intakedroptimer.reset();
                        }

                        if(intake.switch_triggered){
                            lift.setTarget(lift.getPositionL() - 25);
                        }

                        if (intake.switch_triggered && intakedroptimer.seconds() > 0.25) {
                            custom_transfer_target = lift.posL + 825;
                            machine = FSM.TRANSFER;
                        }

                        if(gamepad1.y){
                            machine = FSM.INTAKE_UP;
                        }

                        if(gamepad1.dpad_right || gamepad2.dpad_right){
                            override_lift_update = true;
                            manual_intake = true;
                        }

                        if(manual_intake){
                            if (gamepad2.right_trigger < 0.15 && gamepad2.left_trigger < 0.15) {
                                lift.liftr.setPower(lift.Rf);
                                lift.liftl.setPower(lift.Lf);
                                manual_lift_stage = "1";
                            }

                            if (gamepad2.right_trigger > 0.15 && gamepad2.left_trigger < 0.15) {
                                lift.liftl.setPower(gamepad2.right_trigger * 0.65 + lift.Lf);
                                lift.liftr.setPower(gamepad2.right_trigger * 0.65 + lift.Rf);
                                manual_lift_stage = "2";
                            }

                            if (gamepad2.left_trigger > 0.15 && gamepad2.right_trigger < 0.15) {
                                lift.liftr.setPower(-gamepad2.left_trigger * 0.4 + lift.Rf);
                                lift.liftl.setPower(-gamepad2.left_trigger * 0.4 + lift.Lf);
                                manual_lift_stage = "3";
                            }

                            if (gamepad2.right_trigger > 0.15 && gamepad2.left_trigger > 0.15) {
                                lift.liftr.setPower(lift.Rf);
                                lift.liftl.setPower(lift.Lf);
                                manual_lift_stage = "4";
                            }
                            if(!intake.switch_triggered){
                                manualintaketimer.reset();
                            }
                            if(intake.state == Intake.State.CLOSED && manualintaketimer.seconds() > 0.25){
                                manual_intake = false;
                                override_lift_update = false;
                                custom_transfer_target = lift.posL + 825;
                                machine = FSM.TRANSFER;
                            }
                        }
                        break;


                    case TRANSFER:

                        override_lift_update = false;
                        if(gamepad1.x){
                            machine = FSM.INTAKE_UP;
                        }

                        if (!off_stack) {
                            //lift.setTarget(lift.lift_hold);
                            lift.setTarget(custom_transfer_target);
                        }

                        if (off_stack) {
                            lift.setTarget(custom_transfer_target);
                        }

                        drive_speed_constant = 0.8;

                        if (gamepad2.y) {
                            lift.setTarget(Lift.lift_high);
                            lift_raise_target = lift.lift_high;
                            liftautoadjusttimer.reset();
                            hasReachedTarget = false;
                            lift_high_stall_check_override = false;
                            machine = FSM.RAISE;
                        }
                        if (gamepad2.x) {
                            lift.setTarget(Lift.lift_mid);
                            lift_raise_target = lift.lift_mid;
                            liftautoadjusttimer.reset();
                            hasReachedTarget = false;
                            lift_high_stall_check_override = true;
                            machine = FSM.RAISE;
                        }

                        if (gamepad2.b) {
                            lift.setTarget(lift.lift_low);
                            lift_raise_target = lift.lift_low;
                            hasReachedTarget = false;
                            liftautoadjusttimer.reset();
                            lift_high_stall_check_override = true;
                            machine = FSM.RAISE;
                        }

                        break;


                    case RAISE:

                        drive_speed_constant = 0.4;

                        if (lift.safeToExtend && lift_raise_target != lift.lift_high) {
                            //extension.setTarget(Extension.full_ext);
                            extentionMAG.setTarget(ExtentionMAG.State.EXTENDED);
                        }

                        if(lift_raise_target == lift.lift_high && lift.getPositionL() > 1650){
                            extentionMAG.setTarget(ExtentionMAG.State.EXTENDED);
                        }

                        if(lift.posL > 3100){
                            lift_high_stall_check_override = true;
                        }

                        if(lift_raise_target != lift.lift_high) {

                            if ((lift.state == Lift.State.HOLDING || lift.state == Lift.State.STALLING) && liftautoadjusttimer.seconds() > 0.75 && lift_high_stall_check_override) {
                                hasReachedTarget = true;
                            }
                        }

                        if(lift_raise_target == lift.lift_high && lift.getPositionL() > 3100){
                            hasReachedTarget = true;
                        }

                        if(hasReachedTarget || gamepad2.dpad_left) {

                            override_lift_update = true;

                            controller_trigger_l = gamepad2.left_trigger;
                            controller_trigger_r = gamepad2.right_trigger;

                            if (gamepad2.right_trigger < 0.15 && gamepad2.left_trigger < 0.15) {
                                lift.liftr.setPower(lift.Rf);
                                lift.liftl.setPower(lift.Lf);
                                manual_lift_stage = "1";
                            }

                            if (gamepad2.right_trigger > 0.1 && gamepad2.left_trigger < 0.1) {
                                lift.liftl.setPower(gamepad2.right_trigger);
                                lift.liftr.setPower(gamepad2.right_trigger);
                                manual_lift_stage = "2";
                            }

                            if (gamepad2.left_trigger > 0.15 && gamepad2.right_trigger < 0.15) {
                                lift.liftr.setPower(-gamepad2.left_trigger * 0.4 + lift.Rf);
                                lift.liftl.setPower(-gamepad2.left_trigger * 0.4 + lift.Lf);
                                manual_lift_stage = "3";
                            }

                            if (gamepad2.right_trigger > 0.15 && gamepad2.left_trigger > 0.15) {
                                lift.liftr.setPower(lift.Rf);
                                lift.liftl.setPower(lift.Lf);
                                manual_lift_stage = "4";
                            }
                        }

                        if(gamepad2.a){
                            intake.intake.setPosition(intake.OPEN);
                            intake.target = Intake.Position.OPEN;
                            hasReachedTarget = false;
                        }
                        if((!intake.switch_triggered && intake.state == Intake.State.OPEN) || gamepad2.dpad_down){
                            dropconetimer.reset();
                            machine = FSM.RETURN;
                        }


                        break;

                    case RETURN:

                        extentionMAG.updateSafeToLower(exttimer);

                        if(dropconetimer.seconds() > 0.75){
                            extentionMAG.setTarget(ExtentionMAG.State.RETRACTED);
                        }

                       if(gamepad2.dpad_down){
                           extentionMAG.setTarget(ExtentionMAG.State.RETRACTED);
                       }


                        if (extentionMAG.state == ExtentionMAG.State.RETRACTED || extentionMAG.state == ExtentionMAG.State.MOVING_BACK) {
                            //lift.setTarget(Lift.lift_hold);
                            override_lift_update = true;
                        }

                        if (gamepad2.right_trigger < 0.15 && gamepad2.left_trigger < 0.15) {
                            lift.liftr.setPower(lift.Rf);
                            lift.liftl.setPower(lift.Lf);
                            manual_lift_stage = "1";
                        }

                        if (gamepad2.right_trigger > 0.1 && gamepad2.left_trigger < 0.1) {
                            lift.liftl.setPower(gamepad2.right_trigger);
                            lift.liftr.setPower(gamepad2.right_trigger);
                            manual_lift_stage = "2";
                        }

                        if (gamepad2.left_trigger > 0.15 && gamepad2.right_trigger < 0.15) {
                            lift.liftr.setPower(-gamepad2.left_trigger * 0.4 + lift.Rf);
                            lift.liftl.setPower(-gamepad2.left_trigger * 0.4 + lift.Lf);
                            manual_lift_stage = "3";
                        }

                        if (gamepad2.right_trigger > 0.15 && gamepad2.left_trigger > 0.15) {
                            lift.liftr.setPower(lift.Rf);
                            lift.liftl.setPower(lift.Lf);
                            manual_lift_stage = "4";
                        }

                        if(gamepad2.dpad_up){
                            sensors.reset();
                            lift.setTarget(lift.lift_stack);
                            override_lift_update = false;
                            machine = FSM.INTAKE_UP;
                        }

                        break;


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

                if(gamepad2.right_bumper && gamepad2.left_bumper && gamepad2.right_trigger > 0.9 && gamepad2.left_trigger > 0.9){
                    isAutoControlled = false;
                }

                if(gamepad1.left_bumper){
                    flipper.flipper.setPosition(flipper.RAISED);
                }
                if(gamepad1.right_bumper){
                    flipper.flipper.setPosition(flipper.LOWERED);
                }

                drive.update();
                //extension.update(lift.time_elapsed);
                //extension.extension.setPower(extension.out);
                extentionMAG.update(exttimer);
                extentionMAG.extension.setPower(extentionMAG.out);
                sensors.updateb(act, snsr_loop, loop_max/*, false, true*/);
                lift.updateFeedforwardNew();

                if(!override_lift_update) {
                    lift.liftl.setPower(lift.outL);
                    lift.liftr.setPower(lift.outL);
                }


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
                dashboardTelemetry.addData("lift power: ", lift.outL);
                dashboardTelemetry.addData("lift override: ", override_lift_update);
                dashboardTelemetry.addData("manual intake timer: ", manualinttimer.seconds());
                dashboardTelemetry.addData("has reached target?: ", hasReachedTarget);
                dashboardTelemetry.addData("trigger l: ", controller_trigger_l);
                dashboardTelemetry.addData("trigger r: ", controller_trigger_r);
                dashboardTelemetry.addData("hasReachedTarget: ", hasReachedTarget);
                dashboardTelemetry.addData("manual lift stage: ", manual_lift_stage);
                dashboardTelemetry.update();

                last_mchn = machine;

            }

        }

    }
}
