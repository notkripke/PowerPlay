package org.firstinspires.ftc.teamcode.drive.opmode.Components;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class NewSensors {

    public enum State{
        WITHIN_RANGE,//if reading is in acceptable threshold
        TOO_FAR,//if object has too high a reading
        TOO_CLOSE,//if object has too low a reading
        NO_VIEW,//if no object in sight (reading is way too high)
        NOT_SURE//if no new updates have happened for a while
    }

    public static State in_state = State.NOT_SURE;
    public static State out_state = State.NOT_SURE;
    public static State inSide_state = State.NOT_SURE;

    public Rev2mDistanceSensor dIn, dOut, dSideIn;

    public static double in_dist = 0;
    public static double out_dist = 0;
    public static double in_side_dist = 0;

    //THRESHOLDS

    public static double high_in_thresh = 7.5;
    public static double low_in_thresh = 6.5;
    public static double max_range_in = 18;

    public static double high_out_thresh = 8.5;
    public static double low_out_thresh = 7.5;
    public static double max_range_out = 16;

    public static double high_in_side = 3;
    public static double low_in_side = 2;
    public static double max_range_side = 6.5;

    //--------------------------------------------




    public NewSensors(HardwareMap hardwareMap){
        dIn = hardwareMap.get(Rev2mDistanceSensor.class, "dIn");
        dOut = hardwareMap.get(Rev2mDistanceSensor.class, "dOut");
        dSideIn = hardwareMap.get(Rev2mDistanceSensor.class, "dside");
        in_state = State.NOT_SURE;
        out_state = State.NOT_SURE;
        inSide_state = State.NOT_SURE;
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

    public void reset(){
        in_dist = 0;
        out_dist = 0;
        in_side_dist = 0;
        in_state = State.NOT_SURE;
        out_state = State.NOT_SURE;
        inSide_state = State.NOT_SURE;
    }

    /**
     *
     * @param timer Use
     * @param intake Set as TRUE if to be used for intake operations, FALSE if using outtake
     *
     */

    //public void updateManual(ElapsedTime timer, double waittime, boolean intake){

      //  if(timer.milliseconds() < )




    //}











}
