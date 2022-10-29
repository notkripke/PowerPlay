/*package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import static org.firstinspires.ftc.teamcode.drive.opmode.Components.Lift.*;
import static org.firstinspires.ftc.teamcode.drive.opmode.Components.Intake.*;
import static org.firstinspires.ftc.teamcode.drive.opmode.Components.Lift.State.BASE;
import static org.firstinspires.ftc.teamcode.drive.opmode.Components.Lift.State.HOLDING;
import static org.firstinspires.ftc.teamcode.drive.opmode.Components.Lift.State.LIFTING;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Extension;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Intake;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Lift;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.PosStorage;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.VisionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.nio.channels.Pipe;


@Autonomous(group = "advanced")
public class RedRight extends GorillabotsCentral {

    PosStorage storage = new PosStorage();

    Pose2d startPose = new Pose2d(36, -60, Math.PI/2);

    double last_stack_height = 0;

    double cycles = 0; //counter for number of cycles to be completed
    double max_cycles = 5;//max number of cycles to be done
    //NOTE: 1 CYCLE CONSISTS OF DRIVING TO THE STACK, INTAKING, AND DEPOSITING.
    //THE PRE-LOADED CONE DEPOSIT IS NOT A CYCLE

    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        double last_time = 0;
        double time = 0;

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime delay_timer = new ElapsedTime();

        int field_pos = 3;

        AutoFSM LIFT_FSM = AutoFSM.INIT;


        //TRAJECTORIES------------------------------------------

        double turnAngle1 = Math.toRadians(35);
        double turnAngle2 = Math.toRadians(-40);

        drive.setPoseEstimate(startPose);


        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose) //from start to high
                .lineToConstantHeading(new Vector2d(36, -15))
                .addTemporalMarker(-0.5, () -> lift.setTarget(lift_high))
                .turn(turnAngle1)
                .build();

        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end()) //from high to stack
                .splineToLinearHeading(new Pose2d(60, -12, Math.toRadians(180)), Math.toRadians(180))
                .build();

        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(trajectory2.end())//from stack to high
                .splineToLinearHeading(new Pose2d(36, -15, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(-0.5, () -> lift.setTarget(lift_high))
                .turn(turnAngle2)
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(trajectory3.end())
                .lineToLinearHeading(new Pose2d(12, -18, Math.toRadians(180)))
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(trajectory3.end())
                .lineToLinearHeading(new Pose2d(36, -18, Math.toRadians(180)))
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(trajectory3.end())
                .lineToLinearHeading(new Pose2d(48, -18, Math.toRadians(180)))
                .turn(Math.toRadians(90))
                .build();


        //END TRAJECTORIES---------------------------------------------------------------------


        startVisionProcessing();

        while (!isStarted() && !isStopRequested()){ //vision
            telemetry.addData("Position: ", Pipeline.getAnalysis());
            telemetry.addData("Avg1: ", Pipeline.getAvg1());
            telemetry.update();

            if(Pipeline.getAnalysis() == VisionPipeline.sleeve_pos.one){
                field_pos = 1;
            }

            if(Pipeline.getAnalysis() == VisionPipeline.sleeve_pos.two){
                field_pos = 2;
            }

            if(Pipeline.getAnalysis() == VisionPipeline.sleeve_pos.three){
                field_pos = 3;
            }
        }



        waitForStart();

        if (isStopRequested()) return;

        LIFT_FSM = AutoFSM.HOLDING_CONE;

        SampleMecanumDrive.AutoFSM drive_state = SampleMecanumDrive.AutoFSM.FORWARD1;

        drive.followTrajectorySequenceAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {

            time = timer.seconds();

            lift.time_elapsed = time - last_time;

            switch (drive_state) {
                case FORWARD1://forward from start to high junction

                    if (!drive.isBusy()) {
                        drive_state = SampleMecanumDrive.AutoFSM.HOLD1;
                    }
                    break;


                case HOLD1:

                    LIFT_FSM = AutoFSM.DEPOSIT;
                    break;

                case BACK1:

                    if(!drive.isBusy()){
                        LIFT_FSM = AutoFSM.STACK;
                    }
                    break;

                case HOLD2:

                    if(lift.state == LIFTING && posR > last_stack_height * 1.2){
                        if(lift.safeToExtend){
                            extension.setTarget(Extension.medium_ext);
                        }
                        drive.followTrajectorySequenceAsync(trajectory3);
                        drive_state = SampleMecanumDrive.AutoFSM.FORWARD1;
                    }
                    break;

                case PARK:

                    if(lift.state == HOLDING) {
                        LIFT_FSM = AutoFSM.BASE;

                        if (field_pos == 1) {
                            drive.followTrajectorySequenceAsync(park1);
                        }
                        if (field_pos == 2) {
                            drive.followTrajectorySequenceAsync(park2);
                        }
                        if (field_pos == 3) {
                            drive.followTrajectorySequenceAsync(park3);
                        }
                        drive_state = SampleMecanumDrive.AutoFSM.DONE;
                    }
                        break;

                case DONE:
                    if(!drive.isBusy() && lift.state == BASE){
                        requestOpModeStop();

                    }

            }

            switch (LIFT_FSM){
                case HOLDING_CONE:
                    lift.setTarget(lift_hold);
                    lift.setMaxPower(1);
                    extension.setTarget(Extension.medium_ext);
                    break;

                case DEPOSIT:
                    lift.setTarget(lift_high);
                    lift.setMaxPower(1);

                    if(lift.safeToExtend){
                        extension.setTarget(Extension.full_ext);
                    }

                    if(lift.state == HOLDING && extension.state == Extension.State.EXTENDED_MAX){
                        intake.target = Position.OPEN;
                    }

                    if(intake.state == Intake.State.OPEN){
                        LIFT_FSM = AutoFSM.RETURN;
                        if(cycles < max_cycles) {
                            drive.followTrajectoryAsync(trajectory2);
                            drive_state = SampleMecanumDrive.AutoFSM.BACK1;
                        }
                        if(cycles == max_cycles){
                            drive_state = SampleMecanumDrive.AutoFSM.PARK;
                        }
                    }

                    break;

                case RETURN:
                    if(extension.safeToLower) {
                        extension.setTarget(extension.intake_pos);
                        lift.setTarget(lift_transfer);
                        lift.setMaxPower(1);
                    }
                    break;

                case STACK:
                    lift.setMaxPower(0.7);
                    lift.setTarget(lift_stack);
                    if(switch_triggered){
                        intake.target = Position.CLOSED;
                        LIFT_FSM = AutoFSM.WAIT;
                    }
                    break;

                case WAIT:
                    if(intake.state == Intake.State.CLOSED){
                        LIFT_FSM = AutoFSM.RETURN;
                        drive_state = SampleMecanumDrive.AutoFSM.HOLD2;
                        last_stack_height = posL;
                    }
                    lift.setTarget(posL);
                    lift.setMaxPower(1);
                    break;

                case BASE:
                    lift.setMaxPower(0.6);
                    lift.setTarget(0);

            }

            lift.update();
            lift.liftr.setPower(outR);
            lift.liftl.setPower(outL);
            intake.update();
            drive.update();
            extension.update(lift.time_elapsed);

            last_time = timer.milliseconds();
            lift.time_overall = timer.milliseconds();

        }

    }
}*/