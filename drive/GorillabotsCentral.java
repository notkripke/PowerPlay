package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.Components.Extension;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Intake;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Lift;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Sensors;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.VisionPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.drive.opmode.Components.Lift.*;


import static java.lang.Math.abs;
import static java.lang.Math.max;

//import org.firstinspires.ftc.teamcodeGIT.teamcode.components.RevGyro;


public abstract class GorillabotsCentral extends LinearOpMode {//testing

    public ElapsedTime timer;
    public OpenCvCamera webcam;
    public VisionPipeline Pipeline;
    public Lift lift;
    public Intake intake;
    public Extension extension;
    public Sensors sensors;
    public SampleMecanumDrive drive;

    public void initializeComponents() {
        timer = new ElapsedTime();
        lift = new Lift(hardwareMap, telemetry);
        extension = new Extension(hardwareMap);
        sensors = new Sensors(hardwareMap);
        //intake = new Intake(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
    }


    public void startVisionProcessing() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        //pipeline = new EOCVtest();
        Pipeline = new VisionPipeline();
        webcam.setPipeline(Pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
}