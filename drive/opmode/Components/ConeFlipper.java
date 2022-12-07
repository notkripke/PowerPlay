package org.firstinspires.ftc.teamcode.drive.opmode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ConeFlipper {

    public Servo flipper;

    public static double RAISED = 0.08;
    public static double LOWERED = 0.58;

    public static Position last_target = Position.RAISED;

    public static double target_pos = 0;

    public ConeFlipper(HardwareMap hardwareMap, Telemetry telemetry){
        flipper = hardwareMap.servo.get("flip");
        target = Position.RAISED;
        state = State.RAISED;
    }

    public enum State{
        LOWERED,
        MOVING,
        RAISED
    }

    public enum Position{
        LOWERED,
        RAISED
    }

    public static Position target = Position.LOWERED;
    public static State state = State.LOWERED;

    public void update(){
        if(target == Position.LOWERED && (flipper.getPosition() > LOWERED)){
            target_pos = LOWERED;
            state = State.MOVING;
        }

        if(target == Position.LOWERED && (flipper.getPosition() == LOWERED)){
            target_pos = LOWERED;
            state = State.LOWERED;
        }

        if(target == Position.RAISED && (flipper.getPosition() < RAISED * 0.75)){
            target_pos = RAISED;
            state = State.MOVING;
        }

        if(target == Position.RAISED &&  flipper.getPosition() > RAISED * 0.75){
            target_pos = RAISED;
            state = State.RAISED;
        }

        last_target = target;
    }
}