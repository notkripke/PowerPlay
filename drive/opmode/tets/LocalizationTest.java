package org.firstinspires.ftc.teamcode.drive.opmode.tets;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Disabled
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //Telemetry telemetry1;

        //FtcDashboard dashboard = FtcDashboard.getInstance();

        //telemetry1 = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        drive.setPoseEstimate(new Pose2d(0,0,0));

        waitForStart();

        while (!isStopRequested()) {
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

// Create a vector from the gamepad x/y inputs
// Then, rotate that vector by the inverse of that heading
           // Vector2d input = new Vector2d(
             //       -gamepad1.left_stick_y,
               //     -gamepad1.left_stick_x
            //).rotated(-poseEstimate.getHeading());

// Pass in the rotated input + right stick value for rotation
// Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();
            dashboardTelemetry.addData("Positions: ", drive.getWheelPositions());
            dashboardTelemetry.addData("X: ", drive.getPoseEstimate().getX());
            dashboardTelemetry.addData("Y: ", drive.getPoseEstimate().getY());
            dashboardTelemetry.addData("R: ", drive.getPoseEstimate().getHeading());
            dashboardTelemetry.update();

        }
    }
}
