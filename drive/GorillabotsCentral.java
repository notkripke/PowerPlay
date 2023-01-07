package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.Components.ConeFlipper;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Extension;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Intake;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Lift;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Sensors;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.VisionPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.AprilTagPipeline;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.drive.opmode.Components.Lift.*;


import static java.lang.Math.abs;
import static java.lang.Math.max;

//import org.firstinspires.ftc.teamcodeGIT.teamcode.components.RevGyro;


public abstract class GorillabotsCentral extends LinearOpMode {//testing

    public ElapsedTime timer;
    public OpenCvCamera webcamL;
    public OpenCvCamera webcamR;
    public VisionPipeline Pipeline;
    public Lift lift;
    public Intake intake;
    public Extension extension;
    public Sensors sensors;
    public SampleMecanumDrive drive;
    public ConeFlipper flipper;
    public AprilTagPipeline apipe;

    public static double tagsize = 0.035306;
    public static double fx = 821;
    public static double fy = 821;
    public static double cx = 330;
    public static double cy = 248;

    public void initializeComponents() {
        timer = new ElapsedTime();
        lift = new Lift(hardwareMap, telemetry);
        lift.resetEncoders();
        flipper = new ConeFlipper(hardwareMap, telemetry);
        //extension = new Extension(hardwareMap);
        sensors = new Sensors(hardwareMap);
        intake = new Intake(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        apipe = new AprilTagPipeline(tagsize, fx, fy, cx, cy);
        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
    }

    public void lowerConstaints(double multiplier) {
        DriveConstants.MAX_ACCEL = DriveConstants.MAX_ACCEL * multiplier;
        DriveConstants.MAX_ANG_ACCEL = DriveConstants.MAX_ANG_ACCEL * multiplier;
        DriveConstants.MAX_VEL = DriveConstants.MAX_VEL * multiplier;
        DriveConstants.MAX_ANG_VEL = DriveConstants.MAX_ANG_VEL * multiplier;
    }

    public void resetConstaints() {
        DriveConstants.MAX_ACCEL = 38;
        DriveConstants.MAX_VEL = 40;
        DriveConstants.MAX_ANG_VEL = Math.toRadians(200);
        DriveConstants.MAX_ANG_ACCEL = Math.toRadians(65);
    }


    public void startVisionProcessing() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamL = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcamL"), cameraMonitorViewId);
        //pipeline = new EOCVtest();
        Pipeline = new VisionPipeline();
        webcamL.setPipeline(Pipeline);
        webcamL.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcamL.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public void startAprilProcessLEFT() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamL = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcamL"), cameraMonitorViewId);

        webcamL.setPipeline(apipe);
        webcamL.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcamL.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


    }

    public void startAprilProcessRIGHT() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamR = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcamR"), cameraMonitorViewId);

        webcamR.setPipeline(apipe);
        webcamR.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcamR.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
}
