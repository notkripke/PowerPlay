package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.ExtentionMAG;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Intake;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous(group = "drive")
public class TestR46Sensor extends GorillabotsCentral {

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


    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        lift.resetEncoders();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        drive.setPoseEstimate(new Pose2d(36,-62,Math.PI));

        ElapsedTime intaketime = new ElapsedTime();
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime exttimer = new ElapsedTime();
        ElapsedTime intake_drop_timer = new ElapsedTime();
        ElapsedTime emergency_park_timer = new ElapsedTime();
        ElapsedTime notGoodSensorTimer = new ElapsedTime();

        boolean checkedForBadSensor = false;

        int thing2 = 0;

        ExtentionMAG extentionMAG = new ExtentionMAG(hardwareMap);

        DriveAutoRR drv = DriveAutoRR.INIT;
        LiftAutoRR lft = LiftAutoRR.INIT;

        boolean goingToStack = false;

        boolean sensorInPlace = false;

        boolean doneWithPreviousPath = false;

        boolean act = false;

        int snsr_loop = 0;

        int loop_max = 40;

        boolean emergency_parked = false;

        Lift.State last_state = Lift.State.BASE;

        lowerConstaints(0.65);

        double last_time = 0;

        TrajectorySequence alignToPole1 = drive.trajectorySequenceBuilder(new Pose2d(36,-62,Math.PI))//0,0,0
                //.addTemporalMarker(0.25, () -> lowerConstaints(0.65))
                //.strafeRight(60)
                .lineToLinearHeading(new Pose2d(36, -21.5, Math.PI))
                //.lineToLinearHeading(new Pose2d(31.5, 0, Math.PI))
                //.forward(4.5)
                //.splineToLinearHeading(new Pose2d(27.5, -4.9, Math.toRadians(139)), Math.toRadians(139))
                .addTemporalMarker(4, () -> lift.setTarget(Lift.lift_mid))
                .build();

        TrajectorySequence driveToStack = drive.trajectorySequenceBuilder(alignToPole1.end())
                .setReversed(false)
                //.splineTo(new Vector2d(42, -12), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(35, -24, Math.PI))
                .lineToLinearHeading(new Pose2d(36, -11, Math.PI))
                .lineToLinearHeading(new Pose2d(64, -11, Math.PI))
                .build();

        TrajectorySequence toMiddlePole = drive.trajectorySequenceBuilder(driveToStack.end())
                .setReversed(false)
                .lineToLinearHeading(new Pose2d(24, -12, Math.PI))
                .addTemporalMarker(2.5, () -> lift.setTarget(lift.lift_mid))
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(toMiddlePole.end())
                .lineToLinearHeading(new Pose2d(24, -11, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(180)))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(toMiddlePole.end())
                .lineToLinearHeading(new Pose2d(24, -11, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(180)))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(toMiddlePole.end())
                .lineToLinearHeading(new Pose2d(24, -11, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(58, -12, Math.toRadians(180)))
                .build();

        //lift.setTarget(Lift.lift_stack);

        double cycles = 2;
        double cycles_completed = 0;

        double ok = 400;

        boolean new_cone_grabbed = false;
        boolean dropped_cone = false;
        double field_pos = 2;

        boolean thing = false;

        startVisionProcessing();

        while(!isStarted() && !isStopRequested()){

            telemetry.addData("Position: ", Pipeline.getAnalysis());
            telemetry.addData("Avg1: ", Pipeline.getAvg1());
            telemetry.update();
            field_pos = Pipeline.sleevePositionInt();
        }


        waitForStart();

        intake_drop_timer.reset();

        intake.target = Intake.Position.CLOSED;


        while(!isStopRequested()){

            lift.time_elapsed = timer.time() - last_time;


            switch(lft){

                case INIT:

                    intake.intake.setPosition(intake.CLOSED);

                    if(lift.target == Lift.lift_mid){
                        lft = LiftAutoRR.LIFTTOPOLE;
                    }

                    break;

                case LIFTTOPOLE:

                    if(lift.posL > lift.lift_low){
                        extentionMAG.setTarget(ExtentionMAG.State.EXTENDED);
                        extentionMAG.extension.setPower(1);
                    }

                    if(intake.target == Intake.Position.CLOSED){
                        dropped_cone = false;
                    }

                    if(intake.state == Intake.State.OPEN){ dropped_cone = true; }

                    if((lift.state == Lift.State.HOLDING || lift.state == Lift.State.STALLING) && extentionMAG.state == ExtentionMAG.State.EXTENDED && sensorInPlace){
                        new_cone_grabbed = false;
                        intake.target = Intake.Position.OPEN;
                        intake.intake.setPosition(intake.OPEN);

                        if(!dropped_cone) {
                            intake_drop_timer.reset();
                            cycles_completed += 1;
                        }

                        if(cycles_completed == cycles && intake.state == Intake.State.OPEN && intake_drop_timer.time() >= 1.5){
                            intake_drop_timer.reset();
                            lft = LiftAutoRR.DONE;
                        }
                        if(cycles_completed < cycles && intake.state == Intake.State.OPEN && intake_drop_timer.time() >= 1.5){
                            intake_drop_timer.reset();
                            lft = LiftAutoRR.INTAKE;
                        }
                    }
                    break;

                case INTAKE:

                    if(intake.state == Intake.State.OPEN){
                        //extension.setTarget(Extension.intake_pos);
                        extentionMAG.setTarget(ExtentionMAG.State.RETRACTED);
                    }

                    if(extentionMAG.state == ExtentionMAG.State.RETRACTED && lift.target != 1 && thing2 == 0){
                        lift.setTarget(Lift.lift_stack);
                    }

                   /* if(emergency_park_timer.seconds() > 6){
                        drv = DriveAutoRR.EMERGENCY;
                        lift.setTarget(Lift.lift_stack);
                        lft = LiftAutoRR.EMERGENCY;
                    }*/

                    if(!drive.isBusy() && goingToStack && !new_cone_grabbed && !intake.switch_triggered){
                        lift.setTarget(1);
                    }

                    if(intake.switch_triggered && goingToStack && intake.state != Intake.State.CLOSED && !new_cone_grabbed && !thing){
                        ok = lift.getPositionL();
                        lift.setTarget(ok);
                        intake_drop_timer.reset();
                        emergency_park_timer.reset();
                        new_cone_grabbed = true;
                        intake.intake.setPosition(intake.CLOSED);
                    }

                    if(intake.switch_triggered && lift.target == ok && !thing && new_cone_grabbed){
                        thing2 += 1;
                        if(emergency_park_timer.seconds() > 2 && thing2 == 150){
                            thing = true;
                        }
                    }

                    if((intake.state == Intake.State.CLOSED && intake_drop_timer.seconds() > 0.5) && new_cone_grabbed && thing && thing2 == 150){//lower the 2 and 150 for quick
                        lift.setTarget(Lift.lift_hold + 500);
                        lft = LiftAutoRR.CLEAR;
                        intake.intake.setPosition(intake.CLOSED);
                    }

                    break;

                case CLEAR:

                    if(lift.state == Lift.State.HOLDING && new_cone_grabbed){
                        drive.followTrajectorySequenceAsync(toMiddlePole);
                        drv = DriveAutoRR.RETURNTOPOLE;
                        lft = LiftAutoRR.INIT;
                    }
                    break;

                case DONE:

                    if(intake.state == Intake.State.OPEN){
                        //extension.setTarget(Extension.intake_pos);
                        extentionMAG.setTarget(ExtentionMAG.State.RETRACTED);
                    }

                    if((extentionMAG.state == ExtentionMAG.State.MOVING_BACK || extentionMAG.state == ExtentionMAG.State.RETRACTED) && intake_drop_timer.seconds() >= 2 && drive.isBusy()){
                        lift.setTarget(1);
                    }

                    break;

                case EMERGENCY:

                    if(emergency_parked){
                        lift.setTarget(1);
                    }

                    break;


            }

            switch(drv) {

                case INIT:

                    if(intake_drop_timer.seconds() > .25){
                        lift.setTarget(Lift.lift_stack);
                        drive.followTrajectorySequenceAsync(alignToPole1);
                        drv = DriveAutoRR.ALIGNTOPOLE1;
                        intake.intake.setPosition(intake.CLOSED);
                    }

                    break;

                case ALIGNTOPOLE1:

                    if(!drive.isBusy()){
                        doneWithPreviousPath = true;
                    }

                    if(!sensorInPlace && doneWithPreviousPath){
                        if(sensors.getDistOut() > 5.1){//3.6
                            drive.setWeightedDrivePower(new Pose2d(0.185, 0, 0));
                        }
                        if(sensors.out_dist < 3.8){//2.8
                            drive.setWeightedDrivePower(new Pose2d(-0.185, -0, 0));
                        }

                        if(sensors.out_dist > 3.9 && sensors.out_dist < 5.1){
                            drive.setWeightedDrivePower(new Pose2d(0,0,0));
                            sensorInPlace = true;
                            doneWithPreviousPath = false;
                        }

                        if(sensors.out_dist > 12 && !checkedForBadSensor){
                            checkedForBadSensor = true;
                            drive.setWeightedDrivePower(new Pose2d(0.185, 0, 0));
                            notGoodSensorTimer.reset();
                        }

                        if(checkedForBadSensor && notGoodSensorTimer.seconds() > 0.5){
                            drive.setWeightedDrivePower(new Pose2d(0,0,0));
                            sensorInPlace = true;
                            doneWithPreviousPath = false;
                        }


                    }

                    if(intake.state == Intake.State.OPEN && lft == LiftAutoRR.INTAKE && intake_drop_timer.seconds() >= 0.75){
                        resetConstaints();
                        drive.followTrajectorySequenceAsync(driveToStack);
                        goingToStack = true;
                        drv = DriveAutoRR.GOTOSTACK;
                    }

                    break;


                case GOTOSTACK:
                    break;

                case RETURNTOPOLE:

                    if(!drive.isBusy()){
                        doneWithPreviousPath = true;
                    }

                    if(!sensorInPlace && doneWithPreviousPath){
                        if(sensors.getDistOut() > 5.1){
                            drive.setWeightedDrivePower(new Pose2d(0.185, 0, 0));
                        }
                    if(sensors.out_dist < 2.8){
                        drive.setWeightedDrivePower(new Pose2d(-0.185, 0, 0));
                    }

                    if(sensors.out_dist > 2.8 && sensors.out_dist < 5.1){
                        drive.setWeightedDrivePower(new Pose2d(0,0,0));
                        sensorInPlace = true;
                        doneWithPreviousPath = false;
                    }

                        if(sensors.out_dist > 12 && !checkedForBadSensor){
                            checkedForBadSensor = true;
                            drive.setWeightedDrivePower(new Pose2d(0.185, 0, 0));
                            notGoodSensorTimer.reset();
                        }

                        if(checkedForBadSensor && notGoodSensorTimer.seconds() > 1){
                            drive.setWeightedDrivePower(new Pose2d(0,0,0));
                            sensorInPlace = true;
                            doneWithPreviousPath = false;
                        }
                    }

                    if(intake.state == Intake.State.OPEN && lft == LiftAutoRR.DONE && intake_drop_timer.seconds() >= 0.75){

                        if(field_pos == 1){ drive.followTrajectorySequenceAsync(park1); }
                        if(field_pos == 2){ drive.followTrajectorySequenceAsync(park2); }
                        if(field_pos == 3){ drive.followTrajectorySequenceAsync(park3); }
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
            //extension.update(lift.time_elapsed);
            extentionMAG.update(exttimer);
            extentionMAG.extension.setPower(extentionMAG.out);
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
            dashboardTelemetry.addData("Ext State: ", extentionMAG.state);
            dashboardTelemetry.addData("Ext target: ", extentionMAG.target);
            dashboardTelemetry.addData("safeToLower: ", extentionMAG.safeToLower);
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