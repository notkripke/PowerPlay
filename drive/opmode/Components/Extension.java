package org.firstinspires.ftc.teamcode.drive.opmode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Extension {

    public enum State{
        EXTENDED_MAX,
        MOVING,
        EXTENDED_MED,
        MID,
        INTAKE
    }

    public DcMotor extension;

    public static double target = 0;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    public static double out = 0;

    public static double pos = 0;

    public static double last_pos = 0;

    public static double angular_velo = 0;

    public static double integral_sum = 0;

    public static double integral_sum_limit = 30;

    public static double max_velo = 2;

    public static double last_target = 0.0;

    public static double integral_sum_base = 0;

    public static double last_error = 0;

    public static double last_estimate = 0.0;

    public static double cur_estimate = 0.0;

    public static double weight = 0;

    public static boolean safeToLower = true;

    public static final double intake_pos = -400;
    public static final double mid = -200;
    public static final double medium_ext = 100;
    public static final double full_ext = 250;

    public static double time_elapsed = 0;

    public State state = State.EXTENDED_MED;


    public Extension(HardwareMap hardwareMap){
        extension = hardwareMap.dcMotor.get("extension");
        state = State.EXTENDED_MED;
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getPosition(){
        pos = extension.getCurrentPosition();
        return pos;
    }

    public double getAngularVelo(){
        angular_velo = (pos - last_pos) / time_elapsed;
        return angular_velo;
    }

    public void resetEncoder(){
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTarget(double newTarget){
        target = newTarget;
    }

    public void setMaxPower(double newMax){
        max_velo = newMax;
    }

    public void update(double time){

        time_elapsed = time;

        double error = target - getPosition();

        cur_estimate = (weight * last_estimate) + (1 - weight) * (error - last_error);

        double derivative = cur_estimate / time_elapsed;

        integral_sum += (error * time_elapsed / 2);

        if(integral_sum > integral_sum_limit){
            integral_sum = integral_sum_limit;
        }

        if(integral_sum < -integral_sum_limit){
            integral_sum = -integral_sum_limit;
        }

        if(target != last_target){

            if(target < pos){
                integral_sum = -integral_sum_base;
            }

            if(target > pos){
                integral_sum = integral_sum_base;
            }

            out = (kP * error) + (kI * integral_sum) + (kD * derivative);

            if(out > 2 * max_velo){ out = max_velo; }
            if(out < 2 * -max_velo){ out = -max_velo; }

            last_error = error;
            last_estimate = cur_estimate;

            if(angular_velo > 0.1 || angular_velo < -0.1){
                state = State.MOVING;
            }
            if(angular_velo > -0.1 && angular_velo < 0.1){

                if(target == medium_ext){ state = State.EXTENDED_MED; }
                if(target == full_ext){ state = State.EXTENDED_MAX; }
                if(target == mid){ state = State.MID; }
                if(target == intake_pos){ state = State.INTAKE; }
            }

            if(target != full_ext && target != medium_ext && pos <= target * 0.6){
                safeToLower = true;
            }
            if(target == full_ext || target == medium_ext || pos >= target * 0.6){
                safeToLower = false;
            }

        }
    }
}
