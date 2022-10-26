package org.firstinspires.ftc.teamcode.drive.opmode.Components;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PosStorage {
/*
This class uses static fields to hold the robot's position. This is helpful for transferring the
robot's position between auto and teleop, which can enable use of field-centric driving in teleop


 */
    public static Pose2d current_pos = new Pose2d();
}
