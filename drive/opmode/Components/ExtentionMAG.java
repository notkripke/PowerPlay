package org.firstinspires.ftc.teamcode.drive.opmode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ExtentionMAG {

    public enum State{
        EXTENDED,
        RETRACTED,
        MOVING_FORTH,
        MOVING_BACK
    }

    public DcMotor extension;

    public DigitalChannel exten_switch;

    public static double time_elapsed = 0;

    public State state = State.RETRACTED;
    public State target = State.RETRACTED;

    public double last_non_zero_out = 0;

    public double last_ref = 0;

    public static double override_loop_index = 0;
    public static double override_loop_thresh = 75;//60

    public static double safe_lower_time_thresh = .75;

    public static boolean safeToLower = false;

    public boolean override = false;//Need to override ISM when changing target from reference point
                                    //until mag switch is cleared.

    public State last_state = State.RETRACTED;
    public State last_target = State.RETRACTED;

    public static double out = 0;

    public ExtentionMAG(HardwareMap hardwareMap){
        extension = hardwareMap.dcMotor.get("extension");
        extension.setDirection(DcMotorSimple.Direction.REVERSE);
        exten_switch = hardwareMap.digitalChannel.get("mag");
        exten_switch.setMode(DigitalChannel.Mode.INPUT);
        state = State.RETRACTED;
        target = State.RETRACTED;
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static boolean mag_switch = false;

    public void updateMagSwitch(){
        mag_switch = !exten_switch.getState();
    }

    public boolean getMagSwitch(){
        return mag_switch;
    }

    public void setTarget(State new_target){
        target = new_target;
    }

    public void updateSafeToLower(ElapsedTime timer){

        if(target == State.RETRACTED && last_target != State.RETRACTED){
            timer.reset();
        }

        if(timer.time() > safe_lower_time_thresh){
            safeToLower = false;
        }

        if(timer.time() <= safe_lower_time_thresh){
            safeToLower = true;
        }

        target = last_target;


    }



    public void update(ElapsedTime timer){

        updateMagSwitch();

        override_loop_index += 1;

        if((target == State.RETRACTED && state == State.EXTENDED) || (target == State.EXTENDED && state == State.RETRACTED)){
            if(target != last_target){
                override_loop_index = 0;
            }
        }

        if(target == State.RETRACTED && last_target != State.RETRACTED){
            timer.reset();
        }

        if(override_loop_index > override_loop_thresh){
            override = false;
        }
        if(override_loop_index < override_loop_thresh){
            override = true;
        }

        if(last_non_zero_out > 0 && mag_switch && !override){
            state = State.EXTENDED;
        }
        if(last_non_zero_out > 0 && !mag_switch){
            state = State.MOVING_FORTH;
        }
        if(last_non_zero_out < 0 && mag_switch){
            state = State.RETRACTED;
        }
        if(last_non_zero_out < 0 && !mag_switch == !override){
            state = State.MOVING_BACK;
        }

        if(state != State.EXTENDED && target == State.EXTENDED){
            out = 1;//1
        }
        if(state == State.EXTENDED && target == State.EXTENDED){
            out = 0.2;
        }
        if(state != State.RETRACTED && target == State.RETRACTED){
            out = -1;
        }
        if(state == State.RETRACTED && target == State.RETRACTED){
            out = -0.2;
        }

        if(timer.time() > safe_lower_time_thresh){
            safeToLower = false;
        }

        if(timer.time() <= safe_lower_time_thresh){
            safeToLower = true;
        }

        if(out != 0){
            last_non_zero_out = out;
        }
        last_state = state;
        last_target = target;
    }
}
