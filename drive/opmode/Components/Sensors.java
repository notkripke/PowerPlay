package org.firstinspires.ftc.teamcode.drive.opmode.Components;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sensors {

    public Rev2mDistanceSensor dIn, dOut, dSideIn, dSideOut;

    public static double in_dist = 0;
    public static double out_dist = 0;
    public static double in_side_dist = 0;
    public static double out_side_dist = 0;

    public double intake_range = 4.5;

    public static double max_considerable_dist_x = 7.5; //If sensor reads higher than this, assume its reading
                                                        // other side of robot

    public double intake_range_side_lowthresh = 2;
    public double intake_range_side_highthresh = 3.5;

    public double outtake_range_lowthresh = 2.5;
    public double outtake_range_highthresh = 5;

    public double outtake_range_side_lowthresh = 1.5;
    public double outtake_range_side_highthresh = 3;

    public double idealInY = 4;
    public double idealInX = 3;

    public double idealOutY = 3.25;
    public double idealOutX = 2;

    public static boolean intakeReady = false;
    public static boolean outtakeReady = false;

    public static boolean tooFar = false;
    public static boolean tooClose = false;
    public static boolean tooRight = false;
    public static boolean tooLeft = false;

    public static double adj_power_y = 0;
    public static double adj_power_x = 0;
    //public static double adj_power_offset = 0.05;//MINIMUM REQUIRED POWER TO GET DRIVETRAIN TO MOVE

    public Sensors(HardwareMap hardwareMap){
        dIn = hardwareMap.get(Rev2mDistanceSensor.class, "dIn");
        dOut = hardwareMap.get(Rev2mDistanceSensor.class, "dOut");
        dSideIn = hardwareMap.get(Rev2mDistanceSensor.class, "dside");
    }

    public double getDistIn(){
        in_dist = dIn.getDistance(DistanceUnit.INCH);
        return in_dist;
    }
    public double getDistOut(){
        out_dist = dOut.getDistance(DistanceUnit.INCH);
        return out_dist;
    }

    public double getDistInSide(){
        in_side_dist = dSideIn.getDistance(DistanceUnit.INCH);
        return in_side_dist;
    }

    public double getDistSideOut(){
        out_side_dist = dSideOut.getDistance(DistanceUnit.INCH);
        return out_side_dist;
    }

    public void reset(){
        in_dist = 0;
        out_dist = 0;
        in_side_dist = 0;
        out_side_dist = 0;
        intakeReady = false;
        outtakeReady = false;
    }

    public void update(boolean activated, int loop_inc, int loop, boolean useOut, boolean useIn){

        if(activated){

            if(loop >= loop_inc){


                if(useIn){

                    if(getDistIn() <= intake_range && (getDistInSide() >= intake_range_side_lowthresh && in_side_dist <= intake_range_side_highthresh)){
                        intakeReady = false;
                    }

                    if(in_dist > intake_range || in_side_dist < intake_range_side_lowthresh || in_side_dist > intake_range_side_highthresh){
                        intakeReady = false;
                    }

                }

                if(useOut){

                    if(getDistOut() >= outtake_range_lowthresh && out_dist <= outtake_range_highthresh){
                        outtakeReady = true;
                    }

                    if(out_dist < outtake_range_lowthresh || out_dist > outtake_range_highthresh){
                        outtakeReady = false;
                    }


                }
            }
        }

    }

    public void updateAutoAdj(boolean activated, int loop_inc, int loop, boolean useOut, boolean useIn){

        if(activated){

            if(loop >= loop_inc){


                if(useIn){

                    if(getDistIn() <= intake_range && (getDistInSide() >= intake_range_side_lowthresh && in_side_dist <= intake_range_side_highthresh)){
                        intakeReady = true;
                    }

                    if(in_dist > intake_range || in_side_dist < intake_range_side_lowthresh || in_side_dist > intake_range_side_highthresh){
                        intakeReady = false;

                        if(in_side_dist > max_considerable_dist_x){
                            adj_power_x = 0;
                        }

                        if(in_side_dist < max_considerable_dist_x){
                            adj_power_x = ((in_side_dist - idealInX) / (2 * max_considerable_dist_x));
                        }

                        adj_power_y = (in_dist - idealInY) / 15;
                    }

                }

                if(useOut){

                    if(getDistOut() >= outtake_range_lowthresh && out_dist <= outtake_range_highthresh){
                        outtakeReady = true;
                    }

                    if(out_dist < outtake_range_lowthresh || out_dist > outtake_range_highthresh){
                        outtakeReady = false;

                        if(out_side_dist > max_considerable_dist_x){
                            adj_power_x = 0;
                        }

                        if(out_side_dist < max_considerable_dist_x){
                            adj_power_x = ((out_side_dist - idealOutX) / (2 * max_considerable_dist_x));
                        }

                        adj_power_y = (out_dist - idealOutY) / 15;
                    }


                }
            }
        }

    }

    public void updateAutoAdjTimer(ElapsedTime timer, double minWaitTimeMilli, boolean useOut, boolean useIn) {

        if (timer.milliseconds() > minWaitTimeMilli) {

            if (useIn) {
                if (getDistIn() <= intake_range && (getDistInSide() >= intake_range_side_lowthresh && in_side_dist <= intake_range_side_highthresh)) {
                    adj_power_y = 0;
                    adj_power_x = 0;
                    intakeReady = true;
                }

                if (in_dist > intake_range || in_side_dist < intake_range_side_lowthresh || in_side_dist > intake_range_side_highthresh) {
                    intakeReady = false;

                    if (in_side_dist > max_considerable_dist_x) {
                        adj_power_x = 0;
                    }

                    if (in_side_dist < max_considerable_dist_x) {
                        adj_power_x = ((in_side_dist - idealInX) / (2 * max_considerable_dist_x));
                    }

                    adj_power_y = (in_dist - idealInY) / 15;
                }

            }

            if (useOut) {

                if (getDistOut() >= outtake_range_lowthresh && out_dist <= outtake_range_highthresh) {
                    adj_power_y = 0;
                    adj_power_x = 0;
                    outtakeReady = true;
                }

                if (out_dist < outtake_range_lowthresh || out_dist > outtake_range_highthresh) {
                    outtakeReady = false;

                    if (out_side_dist > max_considerable_dist_x) {
                        adj_power_x = 0;
                    }

                    if (out_side_dist < max_considerable_dist_x) {
                        adj_power_x = ((out_side_dist - idealOutX) / (2 * max_considerable_dist_x));
                    }

                    adj_power_y = (out_dist - idealOutY) / 15;
                }


            }
            timer.reset();
        }

    }

}
