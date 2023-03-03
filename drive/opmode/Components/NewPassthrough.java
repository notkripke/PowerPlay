package org.firstinspires.ftc.teamcode.drive.opmode.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class NewPassthrough {

    public static CRServo servo;
    public static DigitalChannel front_switch;
    public static DigitalChannel back_switch;

    public static boolean in_spring = false;

    public int inc = 0;
    public int inc_cap = 150;

    public static double springback_time = 200;
    public static double springback_power = -0.1;

    public static boolean front_trig = false;//is front switch pressed
    public static boolean back_trig = false;//is back switch pressed

    public static boolean touched = false;
    public static boolean done_spring = false;

    public static double servo_speed_constant = 1;//.setPower() constant
    public static double out = 0;//output crservo power


    public enum State{
        EXTENDED,
        RETRACTED,
        MOVING
    }

    public static State target = State.RETRACTED;
    public static State state = State.RETRACTED;
    public static State last_target = State.RETRACTED;

    public NewPassthrough(HardwareMap hardwareMap){
        servo = hardwareMap.crservo.get("pass");
        front_switch = hardwareMap.digitalChannel.get("front");
        front_switch.setMode(DigitalChannel.Mode.INPUT);
        back_switch = hardwareMap.digitalChannel.get("back");
        back_switch.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean getFrontState(){//updates front switch and returns value. ONLY CALL THIS ONCE PER LOOP

        front_trig = !front_switch.getState();//If pressing the switch results in 0 instead of 1, add !

        return front_trig;
    }

    public boolean getBackState(){

        back_trig = !back_switch.getState();//same applies here

        return back_trig;
    }

    public void setTarget(State newtarget){//sets new target (wow)
        target = newtarget;
    }

    public void update(ElapsedTime timer){

        if(target != last_target){
            touched = false;
            done_spring = false;
            inc = 0;
        }

        if((!touched && front_trig || !touched && back_trig) && inc > inc_cap){
            touched = true;
        }

        if(!touched){
            timer.reset();
        }

        if(touched && !done_spring){
            if(target == State.RETRACTED){
                out = springback_power;
            }
            if(target == State.EXTENDED){
                out = -springback_power;
            }
        }

        if(timer.milliseconds() > springback_time){
            done_spring = true;
        }

        if(getFrontState()){//----------------State updates-------------
            state = State.RETRACTED;
        }

        if(getBackState()){
            state = State.EXTENDED;
        }

        if((!front_trig && !back_trig) || !touched){
            state = State.MOVING;
        }//--------------------------------------------------------------


        if(target == State.EXTENDED && (!touched && !done_spring)){//---------Power updates----------
            out = -servo_speed_constant;
        }

        if(target == State.RETRACTED && (!touched && !done_spring)){
            out = servo_speed_constant;
        }

        if(state == target && done_spring){
            out = 0;
        }//--------------------------------------------------------------

        last_target = target;
        inc += 1;

    }

    public void newUpdate(ElapsedTime timer){

        getFrontState();
        getBackState();

        if(target != last_target){
            touched = false;
            done_spring = false;
            in_spring = false;
            inc = 0;
        }

        if((!touched && front_trig || !touched && back_trig) && inc > inc_cap){
            touched = true;
        }

        if(!touched){
            timer.reset();
        }

        if(front_trig && target == State.RETRACTED){
            out = springback_power;
            in_spring = true;
        }

        if(back_trig && target == State.EXTENDED){
            out = -springback_power;
            in_spring = true;
        }

        if(in_spring && target == State.RETRACTED && !front_trig){
            done_spring = true;
        }

        if(in_spring && target == State.EXTENDED && !back_trig){
            done_spring = true;
        }

        /*if(target == State.RETRACTED && done_spring){//----------------State updates-------------
            state = State.RETRACTED;
        }

        if(target == State.EXTENDED && done_spring){
            state = State.EXTENDED;
        }*/

        if(done_spring){
            state = target;
        }

        if(!done_spring){
            state = State.MOVING;
        }//--------------------------------------------------------------


        if(target == State.EXTENDED && (!touched && !done_spring && !in_spring)){//---------Power updates----------
            out = -servo_speed_constant;
        }

        if(target == State.RETRACTED && (!touched && !done_spring && !in_spring)){
            out = servo_speed_constant;
        }

        if(done_spring){
            out = 0;
        }//--------------------------------------------------------------

        last_target = target;
        inc += 1;

    }




}
