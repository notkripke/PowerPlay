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
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Lift;


@TeleOp(group = "drive")
@Disabled
public class AutoFSMTest extends GorillabotsCentral {

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

        ElapsedTime timer = new ElapsedTime();

        ElapsedTime intaketime = new ElapsedTime();

        double last_time = 0;

        boolean act = false;

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
                    extension.setTarget(Extension.intake_pos);

                    act = true;

                    if(lift.state == Lift.State.HOLDING && intake.state == Intake.State.OPEN){
                        if(sensors.intakeReady){

                            lift.setTarget(1);
                            machine = FSM.INTAKE_DOWN;
                        }
                    }

                    break;

                case INTAKE_DOWN:

                    if(intake.state == Intake.State.CLOSED && intake.switch_cooldown){
                        lift.setTarget(0);
                        act = false;
                        machine = FSM.TRANSFER;
                    }

                    break;


                case TRANSFER:

                    lift.setTarget(lift.lift_hold);

                    if(gamepad1.y){
                        lift.setTarget(Lift.lift_low);
                        machine = FSM.RAISE;
                    }
                    if(gamepad1.x){
                        lift.setTarget(Lift.lift_mid);
                        machine = FSM.RAISE;
                    }

                    if(gamepad1.start){
                        lift.setTarget(lift.lift_high);
                        machine = FSM.RAISE;
                    }

                    break;


                case RAISE:

                    if(lift.safeToExtend){
                        extension.setTarget(Extension.full_ext);
                    }

                    if(lift.state == Lift.State.HOLDING && extension.state == Extension.State.EXTENDED_MAX){
                        intake.override = true;
                        intake.target = Intake.Position.OPEN;
                        machine = FSM.RETURN;
                    }

                    break;

                case RETURN:

                    if(intake.state == Intake.State.OPEN && gamepad1.dpad_right){
                        extension.setTarget(Extension.intake_pos);
                    }
                    if(extension.state == Extension.State.MOVING && extension.safeToLower){
                        lift.setTarget(Lift.lift_hold);
                        machine = FSM.INTAKE_UP;
                    }

                    break;


            }

            if(gamepad1.a){
                intake.target = Intake.Position.OPEN;
            }
            if(gamepad1.b){
                intake.target = Intake.Position.CLOSED;
            }

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
            dashboardTelemetry.addData("Lift state: ", lift.state);
            dashboardTelemetry.addData("Ext. state: ", extension.state);
            dashboardTelemetry.addData("safeToLower?: ", extension.safeToLower);
            dashboardTelemetry.update();



        }

    }
}
