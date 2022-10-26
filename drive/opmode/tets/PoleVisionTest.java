package org.firstinspires.ftc.teamcode.drive.opmode.tets;


import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.VisionPipelinePole;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@TeleOp(group = "AAAAAAAAAA", name = "visionTestPole")

public class PoleVisionTest extends LinearOpMode {
    public void runOpMode() {

        // 192.168.43.1:8080/dash


        CameraStreamSource src = new CameraStreamSource() {
            @Override
            public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {

            }
        };


        VisionPipelinePole Pipeline;

        OpenCvCamera webcam;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        //pipeline = new EOCVtest();
        Pipeline = new VisionPipelinePole();
        webcam.setPipeline(Pipeline);

        webcam.openCameraDevice();

        webcam.setPipeline(Pipeline);

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

            //telemetry.addData("Pole Width (px): ", Pipeline.getPoleWidth());
            //telemetry.addData("Pole Center (1-1270px): ", Pipeline.getPoleCenter());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {

            telemetry.update();
        }

    }

}