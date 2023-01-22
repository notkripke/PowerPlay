package org.firstinspires.ftc.teamcode.drive.opmode.tets;


import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.VisionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(group = "AAAAAAAAAA", name = "visionTest")
public class VisionTest extends LinearOpMode {
    public void runOpMode() {

        // 192.168.43.1:8080/dash


        FtcDashboard dashboard = FtcDashboard.getInstance();

        CameraStreamSource src = new CameraStreamSource() {
            @Override
            public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {

            }
        };

        Telemetry dashboardTelemetry = dashboard.getTelemetry();



        VisionPipeline Pipeline;

        OpenCvCamera webcam;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        //pipeline = new EOCVtest();
        Pipeline = new VisionPipeline();
        webcam.setPipeline(Pipeline);

        webcam.openCameraDevice();

        webcam.setPipeline(Pipeline);

        FtcDashboard.getInstance().startCameraStream(webcam, 24);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        while (!isStarted() && !isStopRequested()){
            dashboardTelemetry.addData("Position: ", Pipeline.getAnalysis());
            dashboardTelemetry.addData("Avg1: ", Pipeline.getAvg1());
            dashboardTelemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {

            telemetry.update();
        }

    }

}