package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.ExtentionMAG;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Intake;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Lift;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.NewPassthrough;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous(group = "drive")
public class RightSpamHigh extends GorillabotsCentral {

    enum DriveAutoRR{
        INIT,
        ALIGNTOPOLE1,
        GOTOSTACK,
        RETURNTOPOLE,
        PARK,
        EMERGENCY
    }

    enum LiftAutoRR{
        INIT,
        LIFTTOPOLE,
        INTAKE,
        CLEAR,
        DONE,
        EMERGENCY
    }

    enum ConePos {
        ONE,
        TWO,
        THREE
    }


    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        lift.resetEncoders();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        drive.setPoseEstimate(new Pose2d(34.5,-62,Math.toRadians(180)));//35x

        ElapsedTime intaketime = new ElapsedTime();
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime exttimer = new ElapsedTime();
        ElapsedTime intake_drop_timer = new ElapsedTime();
        ElapsedTime emergency_park_timer = new ElapsedTime();

        int thing2 = 0;

        boolean safeToClear = false;

        DriveAutoRR drv = DriveAutoRR.INIT;
        LiftAutoRR lft = LiftAutoRR.INIT;

        boolean goingToStack = false;

        boolean act = false;

        int snsr_loop = 0;

        int loop_max = 40;

        Lift.State last_state = Lift.State.BASE;

        lowerConstaints(0.65);

        double last_time = 0;

        TrajectorySequence alignToPole1 = drive.trajectorySequenceBuilder(new Pose2d(34.5,-62,Math.toRadians(180)))//35x
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(37, -20, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(28.5, -5, Math.toRadians(120)), Math.toRadians(120))
                .addTemporalMarker(0.75, () -> lift.setTarget(Lift.lift_high))//3
                .build();

        TrajectorySequence driveToStack = drive.trajectorySequenceBuilder(alignToPole1.end())
                .setReversed(true)//false
                .addTemporalMarker(0.5, () -> lift.setTarget(925))
                .setReversed(true)
                .splineTo(new Vector2d(38, -12), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(62, -12, Math.toRadians(180)))
                .build();

        TrajectorySequence toMiddlePole = drive.trajectorySequenceBuilder(driveToStack.end())
                .setReversed(false)
                //.lineToLinearHeading(new Pose2d(37, -12, Math.PI))//x25.5
                .addTemporalMarker(0.5, () -> lift.setTarget(lift.lift_high))
                .lineToLinearHeading(new Pose2d(43.5, -12, Math.toRadians(180)))
                .splineTo(new Vector2d(29, -3.75), Math.toRadians(130))
                .build();

        TrajectorySequence stackFromCycle = drive.trajectorySequenceBuilder(toMiddlePole.end())
                .setReversed(true)//
                .addTemporalMarker(0.75, () -> lift.setTarget(Lift.lift_stack))
                .splineTo(new Vector2d(38, -12), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(61.75, -12, Math.toRadians(180)))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(toMiddlePole.end())
                .setReversed(true)
                .splineTo(new Vector2d(37, -14), Math.toRadians(10))
                .splineTo(new Vector2d(59.5, -11.75), Math.toRadians(0))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(toMiddlePole.end())
                .setReversed(true)
                .splineTo(new Vector2d(36, -11), Math.toRadians(0))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(toMiddlePole.end())
                .setReversed(true)
                .splineTo(new Vector2d(36, -11), Math.toRadians(0))
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(13.5, -12, Math.toRadians(180)))
                .build();

        //lift.setTarget(Lift.lift_stack);

        double cycles = 4;//3
        double cycles_completed = 0;

        double ok = 400;

        boolean new_cone_grabbed = false;
        boolean dropped_cone = false;
        double field_pos = 2;

        boolean thing = false;

        startAprilProcessRIGHT();


        ConePos conePos = ConePos.TWO;

        final double FEET_PER_METER = 3.28084;

        double last_real_id = 1;

        int numFramesWithoutDetection = 0;

        final float DECIMATION_HIGH = 3;
        final float DECIMATION_LOW = 2;
        final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
        final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

        while(!isStarted() && !isStopRequested()){

            ArrayList<AprilTagDetection> detections = apipe.getDetectionsUpdate();

            // If there's been a new frame...
            if(detections != null)
            {
                telemetry.addData("FPS", webcamR.getFps());
                telemetry.addData("Overhead ms", webcamR.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", webcamR.getPipelineTimeMs());
                telemetry.addData("thing: ", apipe.getDetectionsUpdate());

                // If we don't see any tags
                if(detections.size() == 0)
                {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                    {
                        apipe.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else
                {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    {
                        apipe.setDecimation(DECIMATION_HIGH);
                    }

                    for(AprilTagDetection detection : detections)
                    {
                        if(detection.id == 0 || detection.id == 1 || detection.id == 2){
                            last_real_id = detection.id;
                        }
                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
                        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
                        if(last_real_id == 0){
                            conePos = ConePos.ONE;
                        }
                        if(last_real_id == 1){
                            conePos = ConePos.TWO;
                        }
                        if(last_real_id == 2){
                            conePos = ConePos.THREE;
                        }
                        telemetry.addData("Cone position: ", conePos);
                    }
                }

                telemetry.update();
            }
        }


        waitForStart();

        intake_drop_timer.reset();

        intake.target = Intake.Position.CLOSED;


        while(!isStopRequested()){

            lift.time_elapsed = timer.time() - last_time;


            switch(lft){

                case INIT:

                    intake.intake.setPosition(intake.CLOSED);

                    if(lift.target == Lift.lift_high){
                        safeToClear = false;
                        lft = LiftAutoRR.LIFTTOPOLE;
                    }

                    break;

                case LIFTTOPOLE:

                    if(lift.posL > lift.lift_low){
                        passthrough.setTarget(NewPassthrough.State.EXTENDED);
                    }

                    if(intake.target == Intake.Position.CLOSED){
                        dropped_cone = false;
                    }

                    if(intake.state == Intake.State.OPEN){ dropped_cone = true; }

                    if((lift.state == Lift.State.HOLDING || lift.state == Lift.State.STALLING) && passthrough.state == NewPassthrough.State.EXTENDED && !drive.isBusy()){
                        new_cone_grabbed = false;
                        intake.target = Intake.Position.OPEN;
                        intake.intake.setPosition(intake.OPEN);

                        if(!dropped_cone) {
                            intake_drop_timer.reset();
                            cycles_completed += 1;
                        }

                        if(cycles_completed == cycles && intake.state == Intake.State.OPEN && intake_drop_timer.time() >= 0.5){//1.5
                            intake_drop_timer.reset();
                            safeToClear = true;
                            drv = DriveAutoRR.RETURNTOPOLE;
                            lft = LiftAutoRR.DONE;
                        }
                        if(cycles_completed < cycles && intake.state == Intake.State.OPEN && intake_drop_timer.time() >= 0.5){//1.5
                            intake_drop_timer.reset();
                            safeToClear = true;
                            lft = LiftAutoRR.INTAKE;
                        }
                    }
                    break;

                case INTAKE:

                    if(intake.state == Intake.State.OPEN){
                        passthrough.setTarget(NewPassthrough.State.RETRACTED);
                    }

                    if(passthrough.state == NewPassthrough.State.RETRACTED && lift.target != 1 && thing2 == 0){
                        lift.setTarget(Lift.lift_stack);
                    }

                   /* if(emergency_park_timer.seconds() > 6){
                        drv = DriveAutoRR.EMERGENCY;
                        lift.setTarget(Lift.lift_stack);
                        lft = LiftAutoRR.EMERGENCY;
                    }*/

                    if(!drive.isBusy() && goingToStack && !new_cone_grabbed && !intake.switch_triggered && lift.target == lift.lift_stack){
                        lift.setTarget(1);
                    }

                    if(intake.switch_triggered && goingToStack && !new_cone_grabbed && !thing){
                        ok = lift.getPositionL();
                        lift.setTarget(ok);
                        intake_drop_timer.reset();
                        emergency_park_timer.reset();
                        new_cone_grabbed = true;
                        intake.intake.setPosition(intake.CLOSED);
                    }

                    if(lift.target == ok && !thing && new_cone_grabbed){
                        thing2 += 1;
                        if(emergency_park_timer.seconds() > 0.25 && thing2 == 15){//.seconds() > 2 thing2 75
                            thing = true;
                        }
                    }

                    if((intake.state == Intake.State.CLOSED && intake_drop_timer.seconds() > 0.25) && new_cone_grabbed && thing && thing2 == 15){//lower the 2 and 150 for quick
                        lift.setTarget(Lift.lift_hold + 1050);
                        thing = false;
                        thing2 = 0;
                        lft = LiftAutoRR.CLEAR;
                        intake.intake.setPosition(intake.CLOSED);
                    }

                    break;

                case CLEAR:

                    if(lift.state == Lift.State.HOLDING && new_cone_grabbed){
                        lowerConstaints(1.3);
                        drive.followTrajectorySequenceAsync(toMiddlePole);
                        drv = DriveAutoRR.ALIGNTOPOLE1;
                        lft = LiftAutoRR.INIT;
                    }
                    break;

                case DONE:

                    if(intake.state == Intake.State.OPEN){
                        passthrough.setTarget(NewPassthrough.State.RETRACTED);
                    }

                    if(passthrough.state == NewPassthrough.State.RETRACTED && drive.isBusy()){
                        lift.setTarget(1);
                    }
                    break;


            }

            switch(drv) {

                case INIT:

                    if (intake_drop_timer.seconds() > .75) {
                        lift.setTarget(Lift.lift_stack);
                        drive.followTrajectorySequenceAsync(alignToPole1);
                        drv = DriveAutoRR.ALIGNTOPOLE1;
                        intake.intake.setPosition(intake.CLOSED);
                    }

                    break;

                case ALIGNTOPOLE1:

                    if(intake.state == Intake.State.OPEN && safeToClear){
                        //resetConstaints();
                        if(cycles_completed == 1) {
                            lowerConstaints(0.7);
                            drive.followTrajectorySequenceAsync(driveToStack);
                        }
                        if(cycles_completed > 1){
                            drive.followTrajectorySequenceAsync(stackFromCycle);
                        }
                        goingToStack = true;
                        drv = DriveAutoRR.GOTOSTACK;
                    }

                    break;


                case GOTOSTACK:
                    break;

                case RETURNTOPOLE:

                    if(intake.state == Intake.State.OPEN && lft == LiftAutoRR.DONE){
                        lowerConstaints(1.50);
                        if(last_real_id == 2){ drive.followTrajectorySequenceAsync(park1); }
                        if(last_real_id == 1){ drive.followTrajectorySequenceAsync(park2); }
                        if(last_real_id == 0){ drive.followTrajectorySequenceAsync(park3); }
                        intake_drop_timer.reset();
                        drv = DriveAutoRR.PARK;
                    }
                    break;

                case PARK:
                    if(lift.target == 1 && !drive.isBusy() && (lift.state == Lift.State.HOLDING || lift.state == Lift.State.STALLING)){
                        requestOpModeStop();
                        stop();
                    }

                    break;

               /* case EMERGENCY:

                    if(lift.state == Lift.State.HOLDING && lift.target == lift.lift_stack){

                        if(field_pos == 1){ drive.followTrajectorySequenceAsync(epark1); }
                        if(field_pos == 2){ drive.followTrajectorySequenceAsync(epark2); }
                        if(field_pos == 3){ drive.followTrajectorySequenceAsync(epark3); }
                        intake_drop_timer.reset();
                        drv = DriveAutoRR.PARK;
                    }

                    break;*/


            }

            lift.updateFeedforwardNew();
            lift.liftl.setPower(lift.outL);
            lift.liftr.setPower(lift.outL);
            intake.update(intaketime);
            passthrough.update();
            passthrough.servo.setPower(passthrough.out);
            sensors.update(act, snsr_loop, loop_max);
            drive.update();

            last_time = timer.milliseconds();
            lift.time_overall = timer.milliseconds();
            last_state = lift.state;

            dashboardTelemetry.addData("Lift L: ", lift.getPositionL());
            dashboardTelemetry.addData("Lift R: ", lift.getPositionR());
            dashboardTelemetry.addData("Intake Target: ", intake.target);
            dashboardTelemetry.addData("Intake State: ", intake.state);
            dashboardTelemetry.addData("Intake Pos: ", intake.intake.getPosition());
            dashboardTelemetry.addData("Switch: ", intake.getSwitchState());
            dashboardTelemetry.addData("Distance: ", sensors.in_dist);
            dashboardTelemetry.addData("Within Intake Range?: ", sensors.intakeReady);
            dashboardTelemetry.addData("Lift Target: ", lift.target);
            dashboardTelemetry.addData("Lift state: ", lift.state);
            dashboardTelemetry.addData("Lift Machine: ", lft);
            dashboardTelemetry.addData("Drive MAchine: ", drv);
            dashboardTelemetry.addData("Ext State: ", passthrough.state);
            dashboardTelemetry.addData("Ext target: ", passthrough.target);
            dashboardTelemetry.addData("dropped cone: ", dropped_cone);
            dashboardTelemetry.addData("intake drop: ", intake_drop_timer.time());
            dashboardTelemetry.addData("Cycles completed: ", cycles_completed);
            dashboardTelemetry.addData("timer thingy: ", emergency_park_timer.seconds());
            dashboardTelemetry.addData("thing: ", thing);
            dashboardTelemetry.addData("thing2: ", thing2);
            dashboardTelemetry.update();
        }
    }
}