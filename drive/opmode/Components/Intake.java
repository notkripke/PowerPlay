package org.firstinspires.ftc.teamcode.drive.opmode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {

    public Servo intake;
    public DigitalChannel limit_switch;

    public static double OPEN = 0.6;
    public static double CLOSED = 0.15;

    public static boolean switch_triggered = false;


    public Intake(HardwareMap hardwareMap, Telemetry telemetry){

        intake = hardwareMap.servo.get("intake");
        limit_switch = hardwareMap.digitalChannel.get("limit");
        limit_switch.setMode(DigitalChannel.Mode.INPUT);
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
            switch_triggered = false;
        }
    }

    public boolean getSwitchState() {
        if (limit_switch.getState()) {
            switch_triggered = false;
        }

        if(!limit_switch.getState()){
            switch_triggered = false;
        }
        return switch_triggered;
    }

    public void update(){
        getSwitchState();
        if(target == Position.CLOSED && !switch_triggered){
            intake.setPosition(CLOSED);
            state = State.MOVING;
        }

        if(target == Position.CLOSED && switch_triggered){
            intake.setPosition(CLOSED);
            state = State.CLOSED;
        }

        if(target == Position.OPEN && (switch_triggered || intake.getPosition() < OPEN * 0.75)){
            intake.setPosition(OPEN);
            state = State.MOVING;
        }

        if(target == Position.OPEN && !switch_triggered && intake.getPosition() > OPEN * 0.75){
            intake.setPosition(OPEN);
            state = State.OPEN;
        }
    }


}
