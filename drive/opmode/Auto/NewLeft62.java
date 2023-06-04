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
public class NewLeft62 extends GorillabotsCentral {

    enum DriveAutoRR{
        INIT,
        ALIGNTOPOLE1,
        GOTOSTACK,
        RETURNTOPOLE,
        PARK,
        EMERGENCY
    }

    enum Traj{
        alignToPole1,
        driveToStack,
        toMiddlePole,
        stackFromCycle,
        park
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
        drive.setPoseEstimate(new Pose2d(-34.5,-62,0));//35x

        ElapsedTime intaketime = new ElapsedTime();
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime pass_timer = new ElapsedTime();
        ElapsedTime intake_drop_timer = new ElapsedTime();
        ElapsedTime emergency_park_timer = new ElapsedTime();
        ElapsedTime newtimer = new ElapsedTime();

        int thing2 = 0;

        boolean safeToClear = false;

        DriveAutoRR drv = DriveAutoRR.INIT;
        LiftAutoRR lft = LiftAutoRR.INIT;

        boolean goingToStack = false;

        boolean been_touched = false;

        boolean really_dropped = false;

        boolean act = false;

        int snsr_loop = 0;

        int loop_max = 40;

        boolean pass_override = true;

        Traj current_traj = Traj.alignToPole1;

        Lift.State last_state = Lift.State.BASE;

        //lowerConstaints(0.65);

        double last_time = 0;

        TrajectorySequence alignToPole1 = drive.trajectorySequenceBuilder(new Pose2d(-34.5,-62,0))//35x
                .addTemporalMarker(0.5, () -> lift.setTarget(Lift.lift_high-90))
                //.lineToLinearHeading(new Pose2d(-36, -22, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-35, -52, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-35, -22, Math.toRadians(7)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-35, -14, Math.toRadians(38)), Math.toRadians(90))
                .addDisplacementMarker(0.99, -0.25, () -> lift.setClearToDrop())//-1
                .splineToLinearHeading(new Pose2d(-26.5, -4.9, Math.toRadians(38)), Math.toRadians(38))//x27, y4.25
                .build();

        TrajectorySequence driveToStack = drive.trajectorySequenceBuilder(alignToPole1.end())
                .setReversed(true)
                .addTemporalMarker(0.5, () -> lift.setTarget(Lift.lift_stack))
                .splineToSplineHeading(new Pose2d(-40, -10.5, Math.toRadians(0)), Math.toRadians(180))
                .addDisplacementMarker(0.99, -0.15, () -> lift.setClearToIntake())
                .lineToLinearHeading(new Pose2d(-62, -10, Math.toRadians(0)))
                .build();

        TrajectorySequence toMiddlePole = drive.trajectorySequenceBuilder(new Pose2d(-62.6, -10.5, Math.toRadians(0)))
                .setReversed(false)
                .addTemporalMarker(0.5, () -> lift.setTarget(Lift.lift_mid))
                .addDisplacementMarker(0.99, -0.25, () -> lift.setClearToDrop())
                //.splineToSplineHeading(new Pose2d(-28.75, -21.65, Math.toRadians(337)), Math.toRadians(337))//x27.7y21.15
                .splineTo(new Vector2d(-29.35, -22.45), Math.toRadians(335))
                .build();

        TrajectorySequence stackFromCycle = drive.trajectorySequenceBuilder(toMiddlePole.end())
                .addTemporalMarker(0.75, () -> lift.setTarget(Lift.lift_stack))
                .setReversed(true)
                .addDisplacementMarker(0.99, -0.15, () -> lift.setClearToIntake())
                //.splineTo(new Vector2d(-62, -11.25), Math.toRadians(180))//y11.8\
                .splineToSplineHeading(new Pose2d(-62.6, -10.5, Math.toRadians(0)), Math.toRadians(175))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(toMiddlePole.end())

                .lineToLinearHeading(new Pose2d(-39, -14, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-58.5, -14, 0))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(toMiddlePole.end())
                .lineToLinearHeading(new Pose2d(-39, -14, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-34, -14, 0))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(toMiddlePole.end())
                .lineToLinearHeading(new Pose2d(-39, -14, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-10, -14, 0))
                .build();

        double cycles = 5;//3
        double cycles_completed = 0;

        boolean first_cycle = true;

        boolean thoop = false;

        double ok = 400;

        boolean new_cone_grabbed = false;
        boolean dropped_cone = false;
        double field_pos = 2;

        boolean thing = false;

        boolean force_closed = true;

        startAprilProcessLEFT();


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

                    if(lift.target == Lift.lift_high-90 || lift.target == lift.lift_mid){
                        safeToClear = false;
                        lft = LiftAutoRR.LIFTTOPOLE;
                        dropped_cone = false;
                        really_dropped = false;
                    }

                    break;

                case LIFTTOPOLE:

                    pass_override = true;

                    intake.ignore_switch = false;

                    passthrough.servo.setPower(-1);

                    if(lift.clear_to_drop && !thoop){
                        thoop = true;
                        newtimer.reset();
                    }

                    new_cone_grabbed = false;
                    if((lift.state == Lift.State.HOLDING || lift.state == Lift.State.STALLING) && lift.clear_to_drop)//!drive.isBusy()
                        force_closed = false;
                        intake.target = Intake.Position.OPEN;
                        intake.intake.setPosition(intake.OPEN);
                        intake.switch_triggered = false;

                        if(intake.state == Intake.State.OPEN){ dropped_cone = true; }

                        if(!really_dropped && dropped_cone){
                            intake_drop_timer.reset();
                            //cycles_completed += 1;
                        }

                        if(dropped_cone){
                            really_dropped = true;
                        }

                        if(cycles_completed == cycles -1 && intake.state == Intake.State.OPEN && intake_drop_timer.time() >= 0.25 && thoop && newtimer.seconds() >= 0.25 && !intake.getSwitchState()){//1.5
                            intake_drop_timer.reset();
                            safeToClear = true;
                            lift.setUnclearToDrop();
                            passthrough.setTarget(NewPassthrough.State.RETRACTED);
                            thoop = false;
                            cycles_completed += 1;
                            drv = DriveAutoRR.RETURNTOPOLE;
                            lft = LiftAutoRR.DONE;
                        }

                        if(cycles_completed < cycles -1 && intake.state == Intake.State.OPEN && intake_drop_timer.time() >= 0.25 && thoop && newtimer.seconds() >= 0.25 && !intake.getSwitchState()){//1.5
                            intake_drop_timer.reset();
                            safeToClear = true;
                            lift.setUnclearToDrop();
                            passthrough.setTarget(NewPassthrough.State.RETRACTED);
                            if(!first_cycle){
                                drive.followTrajectorySequenceAsync(stackFromCycle);
                                current_traj = Traj.stackFromCycle;
                            }
                            if(first_cycle){
                                drive.followTrajectorySequenceAsync(driveToStack);
                                current_traj = Traj.driveToStack;
                            }
                            cycles_completed += 1;
                            thoop = false;
                            first_cycle = false;
                            goingToStack = true;
                            lft = LiftAutoRR.INTAKE;
                    }
                    break;

                case INTAKE:

                    //passthrough.setTarget(NewPassthrough.State.RETRACTED);

                    pass_override = true;

                    passthrough.servo.setPower(0.85);

                    if(intake_drop_timer.seconds() >= 0.5 && lift.target != 1 && thing2 == 0 && !thoop){
                        lift.setTarget(Lift.lift_stack);
                        thoop = true;
                    }

                    if(lift.clear_to_intake && goingToStack && !new_cone_grabbed && !intake.switch_triggered && lift.target == lift.lift_stack){
                        intake.ignore_switch = false;
                        lift.setTarget(1);
                        lift.setUnclearToIntake();
                        lift.setUnclearToDrop();
                        thing2 = 0;
                    }

                    if(intake.switch_triggered && goingToStack && !new_cone_grabbed){
                        lift.setUnclearToIntake();
                        ok = lift.getPositionL();
                        lift.setTarget(ok - 100);
                        intake_drop_timer.reset();
                        emergency_park_timer.reset();
                        new_cone_grabbed = true;
                        intake.intake.setPosition(intake.CLOSED);
                        force_closed = true;
                    }

                    if(new_cone_grabbed){
                        thing2 += 1;
                    }

                    if(intake_drop_timer.seconds() > 0.2 && new_cone_grabbed && thing2 >= 25){//lower the 2 and 150 for quick
                        lift.setTarget(940);
                        thing = false;
                        thing2 = 0;
                        lft = LiftAutoRR.CLEAR;
                        thoop = false;
                        intake.intake.setPosition(intake.CLOSED);
                        dropped_cone = false;
                        really_dropped = false;
                    }
                    break;

                case CLEAR:

                    if(lift.state == Lift.State.HOLDING && new_cone_grabbed){
                        //passthrough.setTarget(NewPassthrough.State.EXTENDED);
                        //lowerConstaints(1.3);
                        passthrough.servo.setPower(-1);
                        drive.followTrajectorySequenceAsync(toMiddlePole);
                        current_traj = Traj.toMiddlePole;
                        //lift.setTarget(Lift.lift_mid);
                        drv = DriveAutoRR.ALIGNTOPOLE1;
                        lft = LiftAutoRR.LIFTTOPOLE;
                    }
                    break;

                case DONE:

                    if(intake.state == Intake.State.OPEN){
                        passthrough.setTarget(NewPassthrough.State.RETRACTED);
                        passthrough.servo.setPower(1);
                    }

                    if(passthrough.getFrontState() && !drive.isBusy()){
                        lift.setTarget(1);
                    }
                    break;
            }

            switch(drv) {

                case INIT:

                    if (intake_drop_timer.seconds() > .35) {
                        lift.setTarget(Lift.lift_stack);
                        drive.followTrajectorySequenceAsync(alignToPole1);
                        current_traj = Traj.alignToPole1;
                        drv = DriveAutoRR.ALIGNTOPOLE1;
                        intake.intake.setPosition(intake.CLOSED);
                    }

                    break;

                case ALIGNTOPOLE1:
                    break;

                case GOTOSTACK:
                    break;

                case RETURNTOPOLE:

                    if(intake.state == Intake.State.OPEN && lft == LiftAutoRR.DONE){
                        //lowerConstaints(1.50);
                        if(last_real_id == 0){ drive.followTrajectorySequenceAsync(park1); }
                        if(last_real_id == 1){ drive.followTrajectorySequenceAsync(park2); }
                        if(last_real_id == 2){ drive.followTrajectorySequenceAsync(park3); }
                        current_traj = Traj.park;
                        intake_drop_timer.reset();
                        drv = DriveAutoRR.PARK;
                    }
                    break;

                case PARK:
                    if(lift.target == 1 && !drive.isBusy() && (lift.state == Lift.State.HOLDING || lift.state == Lift.State.STALLING)){
                        //requestOpModeStop();
                        //stop();
                    }

                    break;
            }

            lift.updateFeedforwardNew();
            lift.liftl.setPower(lift.outL);
            lift.liftr.setPower(lift.outL);

            if(force_closed){
                intake.intake.setPosition(intake.CLOSED);
            }
            if(!force_closed) {
                intake.update(intaketime);
            }

            if(!pass_override) {
                passthrough.newUpdate(pass_timer);
                passthrough.servo.setPower(passthrough.out);
            }
            //sensors.update(act, snsr_loop, loop_max);
            drive.update();

            last_time = timer.milliseconds();
            lift.time_overall = timer.milliseconds();
            last_state = lift.state;

            telemetry.addData("clear to drop?: ", lift.clear_to_drop);
            telemetry.addData("Current traj: ", current_traj);
            telemetry.addData("Lift L: ", lift.getPositionL());
            telemetry.addData("Lift R: ", lift.getPositionR());
            telemetry.addData("Is drive busy?: ", drive.isBusy());
            telemetry.addData("Intake Target: ", intake.target);
            telemetry.addData("Intake State: ", intake.state);
            telemetry.addData("Intake Pos: ", intake.intake.getPosition());
            telemetry.addData("Switch: ", intake.getSwitchState());
            telemetry.addData("Lift Target: ", lift.target);
            telemetry.addData("Lift state: ", lift.state);
            telemetry.addData("Lift Machine: ", lft);
            telemetry.addData("Drive Machine: ", drv);
            telemetry.addData("Ext State: ", passthrough.state);
            telemetry.addData("Ext target: ", passthrough.target);
            telemetry.addData("dropped cone: ", dropped_cone);
            telemetry.addData("intake drop: ", intake_drop_timer.time());
            telemetry.addData("Cycles completed: ", cycles_completed);
            telemetry.addData("timer thingy: ", emergency_park_timer.seconds());
            telemetry.addData("thing: ", thing);
            telemetry.addData("thing2: ", thing2);
            telemetry.update();
        }
    }
}