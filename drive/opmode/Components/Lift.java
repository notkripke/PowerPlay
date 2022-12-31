package org.firstinspires.ftc.teamcode.drive.opmode.Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lift {

    public DcMotor liftl, liftr;

    public enum State{
        LIFTING,
        HOLDING,
        LOWERING,
        BASE,
        STALLING
    }

    public enum AutoFSM{
        INIT,
        HOLDING_CONE,
        DEPOSIT,
        RETURN,
        STACK,
        BASE,
        WAIT
    }

    public static State state = State.BASE;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry){
        liftl = hardwareMap.dcMotor.get("liftl");
        liftl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftr = hardwareMap.dcMotor.get("liftr");
        liftr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //liftl.setDirection(DcMotorSimple.Direction.REVERSE);
        liftr.setDirection(DcMotorSimple.Direction.REVERSE);
        init_pos = getPositionL();
        target = init_pos;
        state = State.BASE;

    }

    public static double LkP = .008;
    public static double LkI = 0.002;
    public static double LkD = 0.0;

    public static double RkP = .008;
    public static double RkI = 0.002;
    public static double RkD = 0.0;

    public static double Rf = .15;//left and right additive feedforward constants.
    public static double Lf = .15;

    public static double target = 0.0;

    public static double init_pos = 0.0;

    public static double posL = 0.0;
    public static double posR = 0.0;

    public static double max_power_dwn = -0.25;

    public static double angularVeloL = 0.0;
    public static double angularVeloR = 0.0;

    public static double integral_sumL = 0.0;
    public static double integral_sumR = 0.0;

    public static double integral_sum_limit = 30;

    public static double max_velo = 2;
    //public static double min_velo = 0; //not necessary

    public static double last_target = 0.0;

    public static double integral_sum_base = 0;

    public static double last_errorR = 0;
    public static double last_errorL = 0;

    public static double last_estimateL = 0.0;
    public static double last_estimateR = 0.0;

    public static double cur_estimateL = 0.0;
    public static double cur_estimateR = 0.0;

    public static double weightR = 0;
    public static double weightL = 0;

    public static double time_elapsed = 0;
    public static double time_overall = 0;

    public static final double lift_high = 3300;
    public static final double lift_mid = 2225;//2300 2250
    public static final double lift_low = 1400;
    public static final double lift_hold = 450;
    public static final double lift_stack = 1000;
    public static final double lift_transfer = 1000;

    public double getPositionL(){
        posL = -liftl.getCurrentPosition();
        return posL;
    }

    public double getPositionR(){
        posR = -liftr.getCurrentPosition(); //REMOVE NEGATION IF ENCODER DIRECTION
        return posR;                        //INFLUENCED BY liftr.setDirection(REVERSED)
    }

   public double getAngularVeloL(){
        angularVeloL = (posL - last_posL) / time_elapsed;
        return angularVeloL;
   }

    public void resetEncoders(){
        liftr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTarget(double height){
        target = height;
    }

    public void setMaxPower(double max){
        max_velo = max;
    }

    public static double outL = 0;
    public static double outR = 0;

    public double last_posL = 0;
    public double last_posR = 0;

    public static boolean safeToExtend = true;

    public static boolean safeToDrive = true;

    public static double safeDriveThresh = lift_mid;

    public void update(){

        double errorL = target - (getPositionL() / 145.1);
        double errorR = target - (getPositionR() / 145.1);

        getAngularVeloL();

        cur_estimateR = (weightR * last_estimateR) + (1 - weightR) * (errorR - last_errorR);
        cur_estimateL = (weightL * last_estimateL) + (1 - weightL) * (errorL - last_errorL);

        double derivativeR = cur_estimateR / time_elapsed;
        double derivativeL = cur_estimateL / time_elapsed;

        integral_sumR += (errorR * time_elapsed / 2);
        integral_sumL += (errorL * time_elapsed / 2);

        if(integral_sumL > integral_sum_limit){
            integral_sumL = integral_sum_limit;
        }
        if(integral_sumL < -integral_sum_limit){
            integral_sumL = -integral_sum_limit;
        }

        if(integral_sumR > integral_sum_limit){
            integral_sumR = integral_sum_limit;
        }
        if(integral_sumR < -integral_sum_limit){
            integral_sumR = -integral_sum_limit;
        }

        if(target != last_target){
            if(target < posR){
                integral_sumR = -integral_sum_base;
                integral_sumL = -integral_sum_base;
            }
            if(target > posR){
                integral_sumL = integral_sum_base;
                integral_sumR = integral_sum_base;
            }
            //integral_sumL = integral_sum_base;
            //integral_sumR = integral_sum_base;
        }

        outL = ((LkP * errorL) + (LkI * integral_sumL) + (LkD * derivativeL)) / 2;
        outR = ((RkP * errorR) + (RkI * integral_sumR) + (RkD * derivativeR)) / 2;

       // if(outL > max_velo){ outL = max_velo;}

        if(outL > 2*max_velo){ outL = max_velo; }

        if(outL < 2 * -max_velo){ outL =- max_velo;}

        if(outR >  2 * max_velo){ outR = max_velo;}

        if(outR < 2 * -max_velo){ outR =- max_velo;}

        last_errorL = errorL;
        last_errorR = errorR;

        last_estimateL = cur_estimateL;
        last_estimateR = cur_estimateR;

        if(angularVeloL > 0.15){
            state = State.LIFTING;
        }

        if(angularVeloL > -0.15 && angularVeloL < 0.15 && (posR > target * 0.95 && posR < target * 1.05)){
            state = State.HOLDING;
        }

        if(angularVeloL < -0.1){
            state = State.LOWERING;
        }

        if(angularVeloL > -0.1 && angularVeloL < 0.15 && (posR > target * 0.95 && posR < target * 1.05) && target == init_pos){
            state = State.BASE;
        }

        if(posR >= target * 0.75){
            safeToExtend = true;
        }
        if(posR < target * 0.75){
            safeToExtend = false;
        }


        last_target = target;

        last_posL = posL;
        last_posR = posR;
    }

    public void updateFeedforward(){

        double errorL = target - getPositionL();
        double errorR = target - getPositionR();

        getAngularVeloL();

        cur_estimateR = (weightR * last_estimateR) + (1 - weightR) * (errorR - last_errorR);
        cur_estimateL = (weightL * last_estimateL) + (1 - weightL) * (errorL - last_errorL);

        double derivativeR = cur_estimateR / time_elapsed;
        double derivativeL = cur_estimateL / time_elapsed;

        integral_sumR += (errorR * time_elapsed / 2);
        integral_sumL += (errorL * time_elapsed / 2);

        if(integral_sumL > integral_sum_limit){
            integral_sumL = integral_sum_limit;
        }
        if(integral_sumL < -integral_sum_limit){
            integral_sumL = -integral_sum_limit;
        }

        if(integral_sumR > integral_sum_limit){
            integral_sumR = integral_sum_limit;
        }
        if(integral_sumR < -integral_sum_limit){
            integral_sumR = -integral_sum_limit;
        }

        if(target != last_target){
            if(target < posR){
                integral_sumR = -integral_sum_base;
                integral_sumL = -integral_sum_base;
            }
            if(target > posR){
                integral_sumL = integral_sum_base;
                integral_sumR = integral_sum_base;
            }
            //integral_sumL = integral_sum_base;
            //integral_sumR = integral_sum_base;
        }

        outL = ((LkP * errorL) + (LkI * integral_sumL) + (LkD * derivativeL)) / 2;
        outR = ((RkP * errorR) + (RkI * integral_sumR) + (RkD * derivativeR)) / 2;

        outR += Rf;//Add feedforward constants to output power. May need to have different
        outL += Lf;//feedforward influence for up and down travel respectively.

        if(outL < max_power_dwn){ outL = max_power_dwn; }
        //if(outR < max_power_dwn){ outR = max_power_dwn; }

        // if(outL > max_velo){ outL = max_velo;}

        if(outL > 2 * max_velo){ outL = max_velo; }

        if(outL < 2 * -max_velo){ outL =- max_velo;}

        if(outR >  2 * max_velo){ outR = max_velo;}

        if(outR < 2 * -max_velo){ outR =- max_velo;}

        last_errorL = errorL;
        last_errorR = errorR;

        last_estimateL = cur_estimateL;
        last_estimateR = cur_estimateR;

        if(angularVeloL > 15){
            state = State.LIFTING;
        }

        if(angularVeloL > -15 && angularVeloL < 15 && (posR > target * 0.95 && posR < target * 1.05)){
            state = State.HOLDING;
        }

        if(angularVeloL < -50){
            state = State.LOWERING;
        }

        /*if(angularVeloL > -0.1 && angularVeloL < 0.15 && (posR > target * 0.95 && posR < target * 1.05) && target == init_pos){
            state = State.BASE;
        }*/

        if(posR >= target * 0.75){
            safeToExtend = true;
        }
        if(posR < target * 0.75){
            safeToExtend = false;
        }

        last_target = target;

        last_posL = posL;
        last_posR = posR;
    }


    public void updateFeedforwardNew(){

        double errorL = target - getPositionL();
        double errorR = target - getPositionR();

        getAngularVeloL();

        cur_estimateR = (weightR * last_estimateR) + (1 - weightR) * (errorR - last_errorR);
        cur_estimateL = (weightL * last_estimateL) + (1 - weightL) * (errorL - last_errorL);

        double derivativeR = cur_estimateR / time_elapsed;
        double derivativeL = cur_estimateL / time_elapsed;

        integral_sumR += (errorR * time_elapsed / 2);
        integral_sumL += (errorL * time_elapsed / 2);

        if(integral_sumL > integral_sum_limit){
            integral_sumL = integral_sum_limit;
        }
        if(integral_sumL < -integral_sum_limit){
            integral_sumL = -integral_sum_limit;
        }

        if(integral_sumR > integral_sum_limit){
            integral_sumR = integral_sum_limit;
        }
        if(integral_sumR < -integral_sum_limit){
            integral_sumR = -integral_sum_limit;
        }

        if(target != last_target){
            if(target < posR){
                integral_sumR = -integral_sum_base;
                integral_sumL = -integral_sum_base;
            }
            if(target > posR){
                integral_sumL = integral_sum_base;
                integral_sumR = integral_sum_base;
            }
            //integral_sumL = integral_sum_base;
            //integral_sumR = integral_sum_base;
        }

        outL = ((LkP * errorL) + (LkI * integral_sumL) + (LkD * derivativeL)) / 2;
        outR = ((RkP * errorR) + (RkI * integral_sumR) + (RkD * derivativeR)) / 2;

        outR += Rf;//Add feedforward constants to output power. May need to have different
        outL += Lf;//feedforward influence for up and down travel respectively.

        if(outL < max_power_dwn){ outL = max_power_dwn; }
        //if(outR < max_power_dwn){ outR = max_power_dwn; }

        if((posL > -50 && posL < 150) && target < 20){
            max_power_dwn = -.5;
        }

        if(posL > 250){
            max_power_dwn = -.5;
        }

        // if(outL > max_velo){ outL = max_velo;}

        if(outL > 2 * max_velo){ outL = max_velo; }

        if(outL < 2 * -max_velo){ outL =- max_velo;}

        if(outR >  2 * max_velo){ outR = max_velo;}

        if(outR < 2 * -max_velo){ outR =- max_velo;}


        last_estimateL = cur_estimateL;
        last_estimateR = cur_estimateR;

        /*if(angularVeloL > 15){
            state = State.LIFTING;
        }

        if(angularVeloL > -15 && angularVeloL < 15 && (posR > target * 0.95 && posR < target * 1.05)){
            state = State.HOLDING;
        }

        if(angularVeloL < -50){
            state = State.LOWERING;
        }*/


        if(errorL < 0 && last_errorL - errorL > 20){//ALL THRESHOLDS 20
            state = State.LOWERING;
        }

        if(errorL > 0 && last_errorL - errorL > 20){
            state = State.LIFTING;
        }

        if(last_errorL - errorL < 20 && errorL < 30 && (target != 0 && target != 1)){
            state = State.HOLDING;
        }

        if(last_errorL - errorL < 20 && errorL > 30){
            state = State.STALLING;
        }

        if((target == 1 || target == 0) && last_errorL - errorL < 10 && errorL < 20){
            state = State.HOLDING;
        }

        if((target == 1 || target == 0) && posL > 20){
            outL = -0.5;
        }


        if(posR >= target * 0.75){
            safeToExtend = true;
        }
        if(posR < target * 0.75){
            safeToExtend = false;
        }

        if(posR > safeDriveThresh){
            safeToDrive = false;
        }
        if(posR < safeDriveThresh){
            safeToDrive = true;
        }

        last_errorL = errorL;
        last_errorR = errorR;

        last_target = target;

        last_posL = posL;
        last_posR = posR;
    }

//145.1
}