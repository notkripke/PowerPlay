package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Intake;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Lift;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.NewPassthrough;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous(group = "drive")
public class LeftFiveHigh extends GorillabotsCentral {

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

        boolean advance_from_drop = false;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        drive.setPoseEstimate(new Pose2d(-34.5,-62,0));//35x

        ElapsedTime intaketime = new ElapsedTime();
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime pass_timer = new ElapsedTime();
        ElapsedTime intake_drop_timer = new ElapsedTime();
        ElapsedTime emergency_park_timer = new ElapsedTime();
        ElapsedTime test_timer = new ElapsedTime();

        int thing2 = 0;

        boolean safeToClear = false;

        DriveAutoRR drv = DriveAutoRR.INIT;
        LiftAutoRR lft = LiftAutoRR.INIT;

        boolean goingToStack = false;

        boolean act = false;

        int snsr_loop = 0;

        int loop_max = 40;

        Lift.State last_state = Lift.State.BASE;

        //lowerConstaints(0.65);

        double last_time = 0;

        TrajectorySequence alignToPole1 = drive.trajectorySequenceBuilder(new Pose2d(-34.5,-62,0))//35x
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-38.5, -23, Math.toRadians(0)))
                .addDisplacementMarker(0.975, -1.5, () -> lift.setClearToDrop())
                .splineToSplineHeading(new Pose2d(-29.0, -5.25, Math.toRadians(60)), Math.toRadians(60))//29.5x
                .addTemporalMarker(0.75, () -> lift.setTarget(Lift.lift_high))//3
                .build();

        TrajectorySequence driveToStack = drive.trajectorySequenceBuilder(alignToPole1.end())
                .setReversed(true)//false
                .addDisplacementMarker(0.99, -.25, () -> lift.setClearToIntake())
                .addTemporalMarker(0.15, () -> lift.setTarget(925))
                .setReversed(true)
                .splineTo(new Vector2d(-62.6, -11.5), Math.toRadians(180))//x38
                //.lineToLinearHeading(new Pose2d(-62, -11.5, Math.toRadians(0)))
                .build();

        TrajectorySequence toMiddlePole = drive.trajectorySequenceBuilder(driveToStack.end())
                .setReversed(false)
                //.lineToLinearHeading(new Pose2d(37, -12, Math.PI))//x25.5
                //.addTemporalMarker(0.50, () -> lift.setTarget(lift.lift_high))//o.5
                .addDisplacementMarker(0.01, 4, () -> lift.setTarget(Lift.lift_high))
                //* after 99% of the path (+0 inches) has been completed
                //.lineToLinearHeading(new Pose2d(-43.5, -12, Math.toRadians(0)))
                .addDisplacementMarker(0.99, 0, () -> lift.setClearToDrop())//clear to drop*
                .splineTo(new Vector2d(-25.3, -4.05), Math.toRadians(47))//y3
                .build();

        TrajectorySequence stackFromCycle = drive.trajectorySequenceBuilder(toMiddlePole.end())
                .setReversed(true)//
                .addTemporalMarker(0.15, () -> lift.setTarget(Lift.lift_stack))
                .splineTo(new Vector2d(-62.3, -11.25), Math.toRadians(180))//x-38
                .addDisplacementMarker(0.99, -0.25, () -> lift.setClearToIntake())
                //.lineToLinearHeading(new Pose2d(-61.5, -12, Math.toRadians(0)))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(toMiddlePole.end())
                .setReversed(true)
                .splineTo(new Vector2d(-37, -14), Math.toRadians(170))
                .splineTo(new Vector2d(-59.5, -11.75), Math.toRadians(180))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(toMiddlePole.end())
                .setReversed(true)
                .splineTo(new Vector2d(-36, -11), Math.toRadians(180))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(toMiddlePole.end())
                .setReversed(true)
                .splineTo(new Vector2d(-36, -11), Math.toRadians(180))
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(-13.5, -12, Math.toRadians(0)))
                .build();

        //lift.setTarget(Lift.lift_stack);

        double cycles = 6;//3
        double cycles_completed = 0;

        double ok = 400;

        boolean new_cone_grabbed = false;
        boolean dropped_cone = false;
        double field_pos = 2;

        boolean thing = false;

        startAprilProcessLEFT();


        ConePos conePos = ConePos.TWO;

        final double FEET_PER_METER = 3.28084;

        double last_real_id = 1;

        boolean clear_to_drop = false;

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
                telemetry.addData("FPS", webcamL.getFps());
                telemetry.addData("Overhead ms", webcamL.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", webcamL.getPipelineTimeMs());
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

                    if(lift.posL > lift.lift_low - 550){
                        passthrough.setTarget(NewPassthrough.State.EXTENDED);
                    }

                    //if(intake.state == Intake.State.OPEN){ dropped_cone = true; }

                    if(lift.clear_to_drop){

                        new_cone_grabbed = false;
                        intake.target = Intake.Position.OPEN;
                        intake.intake.setPosition(intake.OPEN);

                        if(!advance_from_drop){
                            test_timer.reset();
                            cycles_completed += 1;
                            advance_from_drop = true;
                        }

                        //if(!dropped_cone) {
                          //  intake_drop_timer.reset();
                           // dropped_cone = true;
                           // cycles_completed += 1;
                        //}

                        if(cycles_completed == cycles && test_timer.seconds() >= 0.65){//1.5
                            intake_drop_timer.reset();
                            lift.setUnclearToDrop();
                            safeToClear = true;
                            goingToStack = true;
                            advance_from_drop = false;
                            drv = DriveAutoRR.RETURNTOPOLE;
                            lft = LiftAutoRR.DONE;
                        }
                        if(cycles_completed < cycles && test_timer.seconds() >= 0.65 && advance_from_drop){//1.5
                            intake_drop_timer.reset();
                            lift.setUnclearToDrop();
                            goingToStack = true;
                            safeToClear = true;
                            advance_from_drop = false;
                            lft = LiftAutoRR.INTAKE;
                        }
                    }
                    break;

                case INTAKE:

                    if(intake.target == Intake.Position.OPEN){
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

                    if(lift.clear_to_intake && goingToStack && !new_cone_grabbed && !intake.switch_triggered && lift.target == lift.lift_stack){
                        lift.setTarget(1);
                        lift.setUnclearToIntake();
                    }

                    if(intake.switch_triggered && goingToStack && !new_cone_grabbed){
                        ok = lift.getPositionL();
                        lift.setTarget(ok-50);
                        intake_drop_timer.reset();
                        emergency_park_timer.reset();
                        new_cone_grabbed = true;
                        intake.intake.setPosition(intake.CLOSED);
                    }

                    if((intake.target == Intake.Position.CLOSED && intake_drop_timer.seconds() > 0.45) && new_cone_grabbed){//lower the 2 and 150 for quick
                        lift.setTarget(Lift.lift_hold + 500);//1050
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

                    if (intake_drop_timer.seconds() > .30) {
                        lift.setTarget(Lift.lift_stack + 400);
                        drive.followTrajectorySequenceAsync(alignToPole1);
                        drv = DriveAutoRR.ALIGNTOPOLE1;
                        intake.intake.setPosition(intake.CLOSED);
                    }

                    break;

                case ALIGNTOPOLE1:

                    passthrough.setTarget(NewPassthrough.State.EXTENDED);

                    if(intake.target == Intake.Position.OPEN && safeToClear){
                        //resetConstaints();
                        if(cycles_completed == 1) {
                            //lowerConstaints(0.7);
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
                        if(last_real_id == 0){ drive.followTrajectorySequenceAsync(park1); }
                        if(last_real_id == 1){ drive.followTrajectorySequenceAsync(park2); }
                        if(last_real_id == 2){ drive.followTrajectorySequenceAsync(park3); }
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
            passthrough.newUpdate(pass_timer);
            passthrough.servo.setPower(passthrough.out);
            sensors.update(act, snsr_loop, loop_max);
            drive.update();

            last_time = timer.milliseconds();
            lift.time_overall = timer.milliseconds();
            last_state = lift.state;

            /*dashboardTelemetry.addData("Lift L: ", lift.getPositionL());
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
            dashboardTelemetry.update();*/
        }
    }
}