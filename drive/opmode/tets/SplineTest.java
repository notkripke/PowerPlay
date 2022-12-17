package org.firstinspires.ftc.teamcode.drive.opmode.tets;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
@Disabled
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        drive.setPoseEstimate(new Pose2d(36,-60,Math.PI/2));


        TrajectorySequence seq = drive.trajectorySequenceBuilder(new Pose2d(36,-60,Math.PI/2))//0,0,0
                .lineToLinearHeading(new Pose2d(36, 0, Math.PI/2))
                .setReversed(true)
                .splineTo(new Vector2d(39, -10), Math.toRadians(300))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(31, -7, Math.toRadians(140)), Math.toRadians(140))
                .setReversed(true)
                .splineTo(new Vector2d(42, -12), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(58, -13, Math.toRadians(180)))
                .setReversed(false)
                .splineTo(new Vector2d(31, -5), Math.toRadians(127))
                .build();




        drive.followTrajectorySequenceAsync(seq);

        waitForStart();

        if (isStopRequested()) return;

        while(!isStopRequested()){


            drive.update();
        }



        sleep(2000);
    }
}
