package org.firstinspires.ftc.teamcode.drive.opmode.tets;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Extension;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Intake;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Lift;
/*
This opmode is designed to test the integration of all the non-locomotion
hardware classes and their synergistic autonomy. This tests the feasibility
of a hyper-automated teleop, in which all key features are automatic aside from
locomotion, height selection, and finalizing intake/outtake operations.
 */

@TeleOp(group = "drive")
@Disabled
public class AutomationTest extends GorillabotsCentral {


    enum FSM{
        START,
        INTAKE,
        WAIT,
        TRANSFER,
        DEPOSIT,
        RETURN
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        double last_time = 0;
        double time = 0;

        int lp = 0;
        int lp_index = 30;

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime intaketime = new ElapsedTime();

        FSM prg_state = FSM.START;

        double last_stack_height = 0;

        boolean sensors_activated = false;

        waitForStart();

        lift.setTarget(Lift.lift_transfer);

        while(!isStopRequested()){

            lp += 1;

            if(lp > lp_index){
                lp = 0;
            }

            time = timer.seconds();

            lift.time_elapsed = time - last_time;

            switch(prg_state){

                case START:
                    intake.target = Intake.Position.OPEN;
                    if(lift.safeToExtend){
                        extension.setTarget(extension.intake_pos);
                    }

                    if(extension.state == Extension.State.INTAKE){
                        prg_state = FSM.INTAKE;
                    }

                    break;


                case INTAKE:

                    intake.target = Intake.Position.OPEN;

                   if(lift.state == Lift.State.HOLDING){

                       sensors_activated = true;

                       if(sensors.intakeReady){
                           lift.setMaxPower(0.5);
                           lift.setTarget(30);

                           if(intake.switch_triggered){
                               last_stack_height = lift.posR;
                               prg_state = FSM.WAIT;
                           }
                       }
                   }
                   break;

                case WAIT:
                    lift.setTarget(last_stack_height);
                    if(intake.state == Intake.State.CLOSED){
                        lift.setMaxPower(1);
                        lift.setTarget(Lift.lift_mid);
                        prg_state = FSM.DEPOSIT;
                    }
                    break;

                case DEPOSIT:
                    lift.setTarget(Lift.lift_mid);
                    if(lift.safeToExtend){
                        extension.setTarget(Extension.full_ext);
                    }
                    if(extension.state == Extension.State.EXTENDED_MAX && lift.state == Lift.State.HOLDING){
                        intake.target = Intake.Position.OPEN;
                        if(intake.state == Intake.State.OPEN){
                            prg_state = FSM.RETURN;
                        }
                    }
                    break;

                case RETURN:
                    extension.setTarget(Extension.intake_pos);
                    if(extension.safeToLower){
                        lift.setTarget(lift.lift_hold);
                    }
                    if(extension.state == Extension.State.INTAKE && lift.state == Lift.State.HOLDING){
                        prg_state = FSM.INTAKE;
                    }

                    break;



            }
            if(prg_state != FSM.INTAKE){
                sensors_activated = false;
            }

            lift.liftr.setPower(lift.outL);
            lift.liftl.setPower(lift.outL);

            intake.intake.setPosition(intake.target_pos);

            extension.extension.setPower(extension.out);

            lift.update();
            intake.update(intaketime);
            extension.update(lift.time_elapsed);
            sensors.update(sensors_activated, lp_index, lp);

            last_time = timer.milliseconds();
            lift.time_overall = timer.milliseconds();

            dashboardTelemetry.addData("Lift L: ", lift.getPositionL());
            dashboardTelemetry.addData("Lift R: ", lift.getPositionR());
            dashboardTelemetry.addData("Intake Target: ", intake.target);
            dashboardTelemetry.addData("Intake State: ", intake.state);
            dashboardTelemetry.addData("Intake Pos: ", intake.intake.getPosition());
            dashboardTelemetry.addData("Extention Pos: ", extension.getPosition());
            dashboardTelemetry.addData("Switch: ", intake.getSwitchState());
            dashboardTelemetry.addData("Distance: ", sensors.in_dist);
            dashboardTelemetry.addData("state: ", prg_state);
            dashboardTelemetry.addData("lift state: ", lift.state);
            dashboardTelemetry.update();


        }
    }
}
