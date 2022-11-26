package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Extension;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.ExtentionMAG;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Intake;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class AutoRR extends GorillabotsCentral {

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

        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        drive.setPoseEstimate(new Pose2d(36,-60,Math.PI/2));

        ElapsedTime intaketime = new ElapsedTime();
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime exttimer = new ElapsedTime();

        ExtentionMAG extentionMAG = new ExtentionMAG(hardwareMap);

        DriveAutoRR drv = DriveAutoRR.INIT;
        LiftAutoRR lft = LiftAutoRR.INIT;

        boolean goingToStack = false;

        boolean act = true;

        int snsr_loop = 0;

        int loop_max = 40;

        Lift.State last_state = Lift.State.BASE;

        double last_time = 0;

        TrajectorySequence alignToPole1 = drive.trajectorySequenceBuilder(new Pose2d(36,-60,Math.PI/2))//0,0,0
                .lineToLinearHeading(new Pose2d(36, 0, Math.PI/2))
                .setReversed(true)
                .splineTo(new Vector2d(39, -10), Math.toRadians(300))
                .setReversed(false)
                .addTemporalMarker(0.1, () -> lift.setTarget(Lift.lift_high))
                .splineToLinearHeading(new Pose2d(31, -7, Math.toRadians(140)), Math.toRadians(140))
                .build();

        TrajectorySequence driveToStack = drive.trajectorySequenceBuilder(alignToPole1.end())
                .setReversed(true)
                .splineTo(new Vector2d(42, -12), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(58, -13, Math.toRadians(180)))
                .build();

        TrajectorySequence backToPole = drive.trajectorySequenceBuilder(driveToStack.end())
                .setReversed(false)
                .splineTo(new Vector2d(31, -5), Math.toRadians(130))
                .addTemporalMarker(-1, () -> lift.setTarget(lift.lift_high))
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

        lift.setTarget(Lift.lift_stack);

        double cycles = 2;
        double cycles_completed = 0;

        boolean new_cone_grabbed = false;
        double field_pos = 2;

        startVisionProcessing();

        while(!isStarted() && !isStopRequested()){

            telemetry.addData("Position: ", Pipeline.getAnalysis());
            telemetry.addData("Avg1: ", Pipeline.getAvg1());
            telemetry.update();
            field_pos = Pipeline.sleevePositionInt();
        }


        waitForStart();


        while(!isStopRequested()){


            switch(lft){

                case INIT:

                    if(lift.target == Lift.lift_high){
                        lft = LiftAutoRR.LIFTTOPOLE;
                    }

                    break;

                case LIFTTOPOLE:

                    if(lift.safeToExtend){
                        extentionMAG.setTarget(ExtentionMAG.State.EXTENDED);
                    }

                    if(lift.state == Lift.State.HOLDING && extentionMAG.state == ExtentionMAG.State.EXTENDED){
                        new_cone_grabbed = false;
                        intake.target = Intake.Position.OPEN;
                        cycles_completed += 1;

                        if(cycles_completed == cycles){
                            lft = LiftAutoRR.DONE;
                        }
                        if(cycles_completed < cycles){
                            lft = LiftAutoRR.INTAKE;
                        }
                    }
                    break;

                case INTAKE:

                    if(intake.state == Intake.State.OPEN){
                        //extension.setTarget(Extension.intake_pos);
                        extentionMAG.setTarget(ExtentionMAG.State.RETRACTED);
                    }

                    if(extentionMAG.safeToLower && !goingToStack){
                        lift.setTarget(Lift.lift_hold);
                    }

                    if(!drive.isBusy() && goingToStack){
                        lift.setTarget(1);
                    }

                    if(intake.switch_triggered && goingToStack && intake.state != Intake.State.CLOSED){
                        lift.setTarget(lift.getPositionL());
                        new_cone_grabbed = true;
                    }

                    if(intake.state == Intake.State.CLOSED && new_cone_grabbed){
                        lift.setTarget(Lift.lift_hold);
                        lft = LiftAutoRR.CLEAR;
                    }

                    break;

                case CLEAR:

                    if(lift.target == lift.lift_hold && lift.state == Lift.State.HOLDING && new_cone_grabbed){
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

                    if(extentionMAG.safeToLower){
                        lift.setTarget(1);
                    }

                    break;


            }

            switch(drv) {

                case INIT:

                    if(lift.state == Lift.State.HOLDING && lift.target == lift.lift_stack){
                        drive.followTrajectorySequenceAsync(alignToPole1);
                        drv = DriveAutoRR.ALIGNTOPOLE1;
                    }

                case ALIGNTOPOLE1:

                    if(intake.state == Intake.State.OPEN && lift.safeToDrive && new_cone_grabbed){
                        drive.followTrajectorySequenceAsync(driveToStack);
                        goingToStack = true;
                        drv = DriveAutoRR.GOTOSTACK;
                    }

                    break;


                case GOTOSTACK:
                    break;

                case RETURNTOPOLE:

                    if(lift.target == 1 && lift.safeToDrive){

                        if(field_pos == 1){ drive.followTrajectorySequenceAsync(park1); }
                        if(field_pos == 2){ drive.followTrajectorySequenceAsync(park2); }
                        if(field_pos == 3){ drive.followTrajectorySequenceAsync(park3); }
                        drv = DriveAutoRR.PARK;
                    }
                    break;

                case PARK:
                    if(!drive.isBusy() && lift.target == 1 && lift.state == Lift.State.HOLDING){
                        requestOpModeStop();
                        stop();
                    }


            }

            lift.updateFeedforwardNew();
            intake.update(intaketime);
            //extension.update(lift.time_elapsed);
            extentionMAG.update(exttimer);
            sensors.update(act, snsr_loop, loop_max);
            drive.update();

            last_time = timer.milliseconds();
            lift.time_overall = timer.milliseconds();
            last_state = lift.state;
        }



    }
}