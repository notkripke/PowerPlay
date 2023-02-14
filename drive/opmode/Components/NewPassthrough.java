package org.firstinspires.ftc.teamcode.drive.opmode.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class NewPassthrough {

    public static CRServo servo;
    public static DigitalChannel front_switch;
    public static DigitalChannel back_switch;

    public static boolean front_trig = false;//is front switch pressed
    public static boolean back_trig = false;//is back switch pressed

    public static double servo_speed_constant = 1;//.setPower() constant
    public static double out = 0;//output crservo power


    public enum State{
        EXTENDED,
        RETRACTED,
        MOVING
    }

    public static State target = State.RETRACTED;
    public static State state = State.RETRACTED;

    public NewPassthrough(HardwareMap hardwareMap){
        servo = hardwareMap.crservo.get("pass");
        front_switch = hardwareMap.digitalChannel.get("front");
        front_switch.setMode(DigitalChannel.Mode.INPUT);
        back_switch = hardwareMap.digitalChannel.get("back");
        back_switch.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean getFrontState(){//updates front switch and returns value. ONLY CALL THIS ONCE PER LOOP

        front_trig = front_switch.getState();//If pressing the switch results in 0 instead of 1, add !

        return front_trig;
    }

    public boolean getBackState(){

        back_trig = back_switch.getState();//same applies here

        return back_trig;
    }

    public void setTarget(State newtarget){//sets new target (wow)
        target = newtarget;
    }

    public void update(){

        if(getFrontState()){//----------------State updates-------------
            state = State.EXTENDED;
        }

        if(getBackState()){
            state = State.RETRACTED;
        }

        if(!front_trig && !back_trig){
            state = State.MOVING;
        }//--------------------------------------------------------------


        if(target == State.EXTENDED){//---------Power updates----------
            out = servo_speed_constant;
        }

        if(target == State.RETRACTED){
            out = -servo_speed_constant;
        }

        if(state == target){
            out = 0;
        }//--------------------------------------------------------------

    }




}
