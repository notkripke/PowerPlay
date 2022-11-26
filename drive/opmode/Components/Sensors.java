package org.firstinspires.ftc.teamcode.drive.opmode.Components;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sensors {

    public Rev2mDistanceSensor dIn, dOut;

    public static double in_dist = 0;
    public static double out_dist = 0;

    public double intake_range = 4.5;

    public static boolean intakeReady = false;

    public Sensors(HardwareMap hardwareMap){
        dIn = hardwareMap.get(Rev2mDistanceSensor.class, "dIn");
        dOut = hardwareMap.get(Rev2mDistanceSensor.class, "dOut");
    }

    public double getDistIn(){
        in_dist = dIn.getDistance(DistanceUnit.INCH);
        return in_dist;
    }

    public void reset(){
        in_dist = 0;
        out_dist = 0;
        intakeReady = false;
    }

    public void update(boolean activated, int loop_inc, int loop){

        if(activated){

            if(loop >= loop_inc){

                if(getDistIn() <= intake_range){
                    intakeReady = true;
                }

                if(in_dist > intake_range){
                    intakeReady = false;
                }
            }
        }

    }

    //public void updateNew(act)




}
