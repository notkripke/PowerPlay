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
import org.firstinspires.ftc.teamcode.drive.opmode.Components.NewPassthrough;


@TeleOp(group = "drive")
public class U2Teleop extends GorillabotsCentral {

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

        //ExtentionMAG extentionMAG = new ExtentionMAG(hardwareMap);

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime exttimer = new ElapsedTime();
        ElapsedTime intaketime = new ElapsedTime();
        ElapsedTime manualinttimer = new ElapsedTime();
        ElapsedTime pass_timer = new ElapsedTime();
        ElapsedTime liftautoadjusttimer = new ElapsedTime();
        ElapsedTime dropconetimer = new ElapsedTime();
        ElapsedTime intakedroptimer = new ElapsedTime();
        ElapsedTime manualintaketimer = new ElapsedTime();

        FSM last_mchn = FSM.INTAKE_UP;

        double lift_raise_target = 0;

        boolean override_lift_update = false;

        boolean last_intake_target_open = true;

       // boolean reintake_watch = false;

        double drive_speed_constant = 1;

        double rotation_power_constant = 0.65;

        double last_time = 0;

        boolean lift_high_stall_check_override = false;

        boolean act = false;

        boolean off_stack = false;

        double custom_transfer_target = 0;

        int snsr_loop = 0;

        int loop_max = 160;//40

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

                        intake.target = Intake.Position.OPEN;
                        intake.intake.setPosition(intake.OPEN);
                        last_intake_target_open = true;
                        passthrough.setTarget(NewPassthrough.State.RETRACTED);
                        drive_speed_constant = 0.9;//1

                        override_lift_update = false;

                        if(!override_lift_update && passthrough.state == NewPassthrough.State.RETRACTED) {
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


                        if (lift.state == Lift.State.HOLDING && intake.state == Intake.State.OPEN && lift.target == lift.lift_stack && lift.last_posL > 500) {

                            act = true;


                            if (sensors.intakeReady && gamepad1.left_trigger < 0.25) {
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

                        passthrough.setTarget(NewPassthrough.State.RETRACTED);

                        lift.max_power_dwn = -0.35;

                        act = false;

                        if(!intake.switch_triggered){
                            intakedroptimer.reset();
                        }

                        if(intake.switch_triggered){
                            lift.setTarget(lift.getPositionL() - 25);
                        }

                        if (intake.switch_triggered && intakedroptimer.seconds() > 0.25) {
                            custom_transfer_target = lift.posL + 975;
                            machine = FSM.TRANSFER;
                        }

                        if(gamepad1.y){
                            sensors.in_dist = 10;
                            sensors.intakeReady = false;
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
                                custom_transfer_target = lift.posL + 975;

                                machine = FSM.TRANSFER;
                            }
                        }
                        break;


                    case TRANSFER:

                        if(lift.posL > custom_transfer_target / 2){
                            passthrough.setTarget(NewPassthrough.State.EXTENDED);
                        }

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

                        drive_speed_constant = 1;

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

                        drive_speed_constant = 0.45;

                        if(lift.posL > custom_transfer_target / 2){
                            passthrough.setTarget(NewPassthrough.State.EXTENDED);
                        }

                        if (lift.safeToExtend && lift_raise_target != lift.lift_high) {
                            passthrough.setTarget(NewPassthrough.State.EXTENDED);
                        }

                        if(lift_raise_target == lift.lift_high && (lift.getPositionL() > 1650 || gamepad2.right_trigger > 0.7)){
                            passthrough.setTarget(NewPassthrough.State.EXTENDED);
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

                        if(dropconetimer.seconds() > 0.25){
                            passthrough.setTarget(NewPassthrough.State.RETRACTED);
                        }

                       if(gamepad2.dpad_up){
                           passthrough.setTarget(NewPassthrough.State.RETRACTED);
                       }


                        if (passthrough.state == NewPassthrough.State.RETRACTED || passthrough.state == NewPassthrough.State.MOVING) {
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
                                -gamepad1.right_stick_x * 0.75
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
                passthrough.newUpdate(pass_timer);

                if(passthrough.getFrontState() && passthrough.target != NewPassthrough.State.EXTENDED){
                    passthrough.out = 0;
                }

                passthrough.servo.setPower(passthrough.out);
                sensors.updateb(act, snsr_loop, loop_max);
                lift.updateFeedforwardNew();

                if(!override_lift_update) {
                    lift.liftl.setPower(lift.outL);
                    lift.liftr.setPower(lift.outL);
                }


                last_time = timer.time();

                /*dashboardTelemetry.addData("Ext State: ", passthrough.state);
                dashboardTelemetry.addData("Ext target: ", passthrough.target);
                dashboardTelemetry.addData("back pass thing: ", passthrough.front_trig);
                dashboardTelemetry.addData("front pass thing: ", passthrough.back_trig);
                dashboardTelemetry.addData("passthrough speed: ", passthrough.servo.getPower());
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
                dashboardTelemetry.addData("lift power: ", lift.outL);
                dashboardTelemetry.addData("lift override: ", override_lift_update);
                dashboardTelemetry.addData("manual intake timer: ", manualinttimer.seconds());
                dashboardTelemetry.addData("has reached target?: ", hasReachedTarget);
                dashboardTelemetry.addData("trigger l: ", controller_trigger_l);
                dashboardTelemetry.addData("trigger r: ", controller_trigger_r);
                dashboardTelemetry.addData("hasReachedTarget: ", hasReachedTarget);
                dashboardTelemetry.addData("manual lift stage: ", manual_lift_stage);
                dashboardTelemetry.addData("distance: ", sensors.in_dist);
                dashboardTelemetry.update();*/

                last_mchn = machine;

            }

        }

    }
}
