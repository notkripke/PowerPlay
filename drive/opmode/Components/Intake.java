package org.firstinspires.ftc.teamcode.drive.opmode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {

    public Servo intake;
    public DigitalChannel limit_switch;


    public static double OPEN = 0.3;//1
    public static double CLOSED = 0.0;

    public static double testpos = 0;

    public static Position last_target = Position.CLOSED;

    public static double target_pos = 0;

    public static boolean switch_triggered = false;

    public static boolean switch_cooldown = false;

    public static double switch_time = 0;
    public static double switch_time_thresh = .3;


    public static boolean last_switch = false;

    public static boolean override = false;


    public Intake(HardwareMap hardwareMap, Telemetry telemetry){

        intake = hardwareMap.servo.get("intake");
        limit_switch = hardwareMap.digitalChannel.get("limit");
        //limit_switch = hardwareMap.get("limit", DigitalChannel.class)
        limit_switch.setMode(DigitalChannel.Mode.INPUT);
        //limit_switch.setMode(DigitalChannel.Mode.OUTPUT);
        target = Position.CLOSED;
        state = State.CLOSED;

    }

    public enum State{
        CLOSED,
        MOVING,
        OPEN
    }

    public enum Position{
        CLOSED,
        OPEN
    }

    public static Position target = Position.CLOSED;
    public static State state = State.CLOSED;

    public void updateSwitchState() {
        if (limit_switch.getState()) {
            switch_triggered = false;
        }

        if(!limit_switch.getState()){
            switch_triggered = true;
        }
    }

    public boolean getSwitchState() {
        if (limit_switch.getState()) {
            switch_triggered = false;
        }

        if(!limit_switch.getState()){
            switch_triggered = true;
        }
        return switch_triggered;
    }

    public static boolean ignore_switch = false;

    public void update(ElapsedTime intaketime){

        if(!ignore_switch) {
            getSwitchState();
        }
        if(target == Position.CLOSED && (intake.getPosition() > CLOSED * 1.35)){
            //intake.setPosition(CLOSED);
            target_pos = CLOSED;
            state = State.MOVING;
        }

        if(target == Position.CLOSED && (intake.getPosition() < CLOSED * 1.35)){
            //intake.setPosition(CLOSED);
            target_pos = CLOSED;
            state = State.CLOSED;
        }

        if(target == Position.OPEN && (intake.getPosition() < OPEN * 0.75)){
            //intake.setPosition(OPEN);
            target_pos = OPEN;
            state = State.MOVING;
        }

        if(target == Position.OPEN &&  intake.getPosition() > OPEN * 0.75){
            //intake.setPosition(OPEN);
            target_pos = OPEN;
            state = State.OPEN;
        }

        if(target != last_target){
            override = true;
        }
        if(target == target){
            override = false;
        }

        if(switch_triggered && target == Position.OPEN){
            target = Position.CLOSED;
        }

        if(switch_triggered && target == Position.CLOSED && override){
            target = Position.OPEN;
        }
            last_target = target;

        if(intaketime.time() > switch_time_thresh){
            switch_cooldown = true;
        }

        if(intaketime.time() < switch_time_thresh){
            switch_cooldown = false;
        }

        if(switch_triggered != last_switch && switch_triggered){
            intaketime.reset();
        }

        last_switch = switch_triggered;

    }

    public void updateAuto(){
        getSwitchState();



    }


}
