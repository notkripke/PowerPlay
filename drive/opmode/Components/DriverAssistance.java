package org.firstinspires.ftc.teamcode.drive.opmode.Components;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class DriverAssistance {

    public static Pose2d resultantVector = new Pose2d();

    public static double smoothingConstant = 0.3;//percentage of desired magnitude applied if below velo thresh
    public static double smoothingThreshold = 24;//velo in/s below which acceleration is controlled

    public static double stoppingMagnitude = 0.25;//power at which the robot will combat its inertia while stopping
    public static double stoppingThreshJoy = 0.1;//percentage of joystick actuation at which stopping is implied
    public static double stoppingThreshVelo = 6;//velo in/s at which robot will cease active stopping procedure

    public static boolean activeStopping = false;

    public static double veloX = 0;
    public static double veloY = 0;
    public static double veloR = 0;

    public static double wantedX = 0;
    public static double wantedY = 0;
    public static double wantedR = 0;

    public static double resultantX = 0;
    public static double resultantY = 0;
    public static double resultantR = 0;

    public static double currTheta = 0;
    public static double wantedTheta = 0;
    public static double resultantTheta = 0;

    public static double wantedMagnitude = 0;
    public static double currMagnitude = 0;

    public static double refR = 0;

    public static double currR = 0;

    public static Pose2d getResultantVector(){

        currTheta = Math.atan(veloY / veloX);

        currMagnitude = Math.sqrt((veloX * veloX) + (veloY * veloY));

        wantedTheta = Math.atan(wantedY / wantedX);

        wantedMagnitude = Math.sqrt((wantedX * wantedX) + (wantedY * wantedY));

        if(wantedMagnitude < stoppingThreshJoy && currMagnitude > stoppingThreshVelo){
            wantedMagnitude = stoppingMagnitude;
            activeStopping = true;
        }

        if(wantedMagnitude > stoppingThreshJoy || currMagnitude < stoppingThreshVelo){
            activeStopping = false;
        }

        if(activeStopping){
            wantedTheta = currTheta + Math.PI;
        }

        if(!activeStopping) {
            resultantTheta = (wantedTheta - currTheta) + wantedTheta;
        }

        resultantX = wantedMagnitude * Math.cos(resultantTheta);
        resultantY = wantedMagnitude * Math.sin(resultantTheta);

        if(veloR > 0.15 || currR - refR > 0.1){
            resultantR = -0.15;
        }
        if(veloR < -0.15 || currR - refR < -0.1){
            resultantR = 0.15;
        }

        resultantVector = new Pose2d(resultantX, resultantY, resultantR);

        return resultantVector;

    }

    public static Pose2d getSmoothEnhancement(boolean negate){
        getResultantVector();

        if(currMagnitude < smoothingThreshold && !activeStopping){
            resultantX = resultantX * smoothingConstant;
            resultantY = resultantY * smoothingConstant;
        }

        if(!negate) {
            resultantVector = new Pose2d(resultantX, resultantY, resultantR);
        }

        if(negate){
            resultantVector = new Pose2d(-resultantX, -resultantY, -resultantR);
        }

        return resultantVector;
    }



}
