package org.firstinspires.ftc.teamcode.drive.opmode.tets;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
public class AutomationTest extends GorillabotsCentral {


    enum FSM{
        INTAKE,
        WAIT,
        TRANSFER,
        DEPOSIT,
        RETURN
    }


    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        FSM prg_state = FSM.INTAKE;

        double last_stack_height = 0;

        waitForStart();

        lift.setTarget(Lift.lift_transfer);

        while(!isStopRequested()){

            switch(prg_state){

                case INTAKE:

                    intake.target = Intake.Position.OPEN;

                   if(lift.state == Lift.State.HOLDING){

                       if(sensors.intakeReady){
                           lift.setMaxPower(0.5);
                           lift.setTarget(Lift.lift_stack);

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



            }
        }
    }
}
