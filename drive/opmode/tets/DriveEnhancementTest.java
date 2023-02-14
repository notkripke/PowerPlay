package org.firstinspires.ftc.teamcode.drive.opmode.tets;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;

@TeleOp(group = "drive")
public class DriveEnhancementTest extends GorillabotsCentral {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        boolean trigger = false;


        while(!isStopRequested()){

            edrive.wantedX = gamepad1.left_stick_x;
            edrive.wantedY = gamepad1.left_stick_y;
            edrive.wantedR = gamepad1.right_stick_x;


            if(gamepad1.left_trigger > 0.3) {
                edrive.veloX = drive.getPoseVelocity().getX();
                edrive.veloY = drive.getPoseVelocity().getY();
                edrive.veloR = drive.getExternalHeadingVelocity();

                if(!trigger){
                    edrive.refR = drive.getPoseEstimate().getHeading();
                }

                edrive.currR = drive.getPoseEstimate().getHeading();

                drive.setWeightedDrivePower(edrive.getSmoothEnhancement(true));

                trigger = true;
            }

            if(gamepad1.left_trigger < 0.3){
                trigger = false;
                drive.setWeightedDrivePower(new Pose2d(-edrive.wantedY, -edrive.wantedX, -edrive.wantedR));
            }



            drive.update();
        }


    }
}
