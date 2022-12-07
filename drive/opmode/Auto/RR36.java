package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Extension;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.ExtentionMAG;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Intake;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class RR36 extends GorillabotsCentral {

    enum DriveAutoRR{
        INIT,
        ALIGNTOPOLE1,
        GOTOSTACK,
        RETURNTOPOLE,
        PARK
    }

    enum LiftAutoRR{
        INIT,
        LIFTTOPOLE,
        INTAKE,
        CLEAR,
        DONE
    }


    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        lift.resetEncoders();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        drive.setPoseEstimate(new Pose2d(36,-60,Math.PI/2));

        ElapsedTime intaketime = new ElapsedTime();
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime exttimer = new ElapsedTime();
        ElapsedTime intake_drop_timer = new ElapsedTime();

        ExtentionMAG extentionMAG = new ExtentionMAG(hardwareMap);

        DriveAutoRR drv = DriveAutoRR.INIT;
        LiftAutoRR lft = LiftAutoRR.INIT;

        boolean goingToStack = false;

        boolean act = false;

        int snsr_loop = 0;

        int loop_max = 40;

        Lift.State last_state = Lift.State.BASE;

        double last_time = 0;

        /*TrajectorySequence alignToPole1 = drive.trajectorySequenceBuilder(new Pose2d(36,-60,Math.PI/2))//0,0,0
                .lineToLinearHeading(new Pose2d(37, 5, Math.PI/2))
                .lineToLinearHeading(new Pose2d(37, -9, Math.toRadians(135)))//125
                .lineToLinearHeading(new Pose2d(27.2, -3, Math.toRadians(124)))
                .lineToLinearHeading(new Pose2d(28, -3, Math.toRadians(124)))
                //.splineToLinearHeading(new Pose2d(27.5, -4.9, Math.toRadians(139)), Math.toRadians(139))
                .addTemporalMarker(6, () -> lift.setTarget(Lift.lift_high))

                .build();*/

        TrajectorySequence alignToPole1 = drive.trajectorySequenceBuilder(new Pose2d(36,-60,Math.PI/2))//0,0,0
                .addTemporalMarker(3, () -> lowerConstaints(0.8))
                .lineToLinearHeading(new Pose2d(37, -5, Math.PI/2))
                .lineToLinearHeading(new Pose2d(37, -12.5, Math.PI/2))//125
                .turn(Math.toRadians(135))
                .lineToLinearHeading(new Pose2d(27.9, -20, Math.toRadians(215)))
                //.splineToLinearHeading(new Pose2d(27.5, -4.9, Math.toRadians(139)), Math.toRadians(139))
                .addTemporalMarker(6, () -> lift.setTarget(Lift.lift_mid))

                .build();

        TrajectorySequence driveToStack = drive.trajectorySequenceBuilder(alignToPole1.end())
                .setReversed(true)
                //.splineTo(new Vector2d(42, -12), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(39, -12.5, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(60.8, -12, Math.toRadians(175.75)))
                .lineToLinearHeading(new Pose2d(62.5, -12, Math.toRadians(174.75)))
                .build();

        TrajectorySequence backToPole = drive.trajectorySequenceBuilder(driveToStack.end())
                .setReversed(false)
                .splineTo(new Vector2d(30.7, -20), Math.toRadians(215))
                //.lineToLinearHeading(new Pose2d(20, -4.2, Math.toRadians(110)))
                .addTemporalMarker(-1, () -> lift.setTarget(lift.lift_mid))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(backToPole.end())
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(12, -12, Math.toRadians(180)))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(backToPole.end())
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(180)))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(backToPole.end())
                .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(58, -12, Math.toRadians(180)))
                .build();

        //lift.setTarget(Lift.lift_stack);

        double cycles = 2;
        double cycles_completed = 0;

        boolean new_cone_grabbed = false;
        boolean dropped_cone = false;
        double field_pos = 2;

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

                    if(lift.posL > lift.lift_mid){
                        extentionMAG.setTarget(ExtentionMAG.State.EXTENDED);
                        extentionMAG.extension.setPower(1);
                    }

                    if(intake.target == Intake.Position.CLOSED){
                        dropped_cone = false;
                    }

                    if(intake.state == Intake.State.OPEN){ dropped_cone = true; }

                    if((lift.state == Lift.State.HOLDING || lift.state == Lift.State.STALLING) && extentionMAG.state == ExtentionMAG.State.EXTENDED){
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

                    if(extentionMAG.state == ExtentionMAG.State.RETRACTED && lift.target != 1){
                        lift.setTarget(Lift.lift_stack);
                    }

                    if(!drive.isBusy() && goingToStack){
                        lift.setTarget(1);
                    }

                    if(intake.switch_triggered && goingToStack && intake.state != Intake.State.CLOSED && !new_cone_grabbed){
                        lift.setTarget(lift.getPositionL());
                        intake_drop_timer.reset();
                        new_cone_grabbed = true;
                        intake.intake.setPosition(intake.CLOSED);
                    }

                    if((intake.state == Intake.State.CLOSED && intake_drop_timer.seconds() > 1.5) && new_cone_grabbed){
                        lift.setTarget(Lift.lift_hold + 500);
                        lft = LiftAutoRR.CLEAR;
                        intake.intake.setPosition(intake.CLOSED);
                    }

                    break;

                case CLEAR:

                    if(lift.state == Lift.State.HOLDING && new_cone_grabbed){
                        drive.followTrajectorySequenceAsync(backToPole);
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


            }

            switch(drv) {

                case INIT:

                    if(intake_drop_timer.seconds() > .25){
                        lift.setTarget(Lift.lift_stack);
                        drive.followTrajectorySequenceAsync(alignToPole1);
                        drv = DriveAutoRR.ALIGNTOPOLE1;
                        intake.intake.setPosition(intake.CLOSED);
                    }

                case ALIGNTOPOLE1:

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
            dashboardTelemetry.update();
        }



    }
}