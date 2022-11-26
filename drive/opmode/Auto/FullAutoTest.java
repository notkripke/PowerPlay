/*package org.firstinspires.ftc.teamcode.drive.opmode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Extension;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Intake;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


@TeleOp(group = "drive")
public class FullAutoTest extends GorillabotsCentral {

    enum FSM{
        INTAKE_UP,
        INTAKE_DOWN,
        TRANSFER,
        RAISE,
        RETURN
    }

    enum DRIVEFSM{
        PRELOAD,
        INTAKE,
        OUTTAKE,
        PARK
    }

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        initializeComponents();

        ElapsedTime timer = new ElapsedTime();

        ElapsedTime intaketime = new ElapsedTime();

        double last_time = 0;

        boolean act = false;

        int snsr_loop = 0;

        int loop_max = 40;

        lift.setTarget(lift.lift_hold);

        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        drive.setPoseEstimate(new Pose2d(0,0,0));

        intake.target = Intake.Position.OPEN;
        intake.last_target = Intake.Position.OPEN;

        double feedforward_cnst = .2;

        FSM machine = FSM.INTAKE_UP;

        waitForStart();

        lift.setTarget(lift.lift_hold);

        intake.target = Intake.Position.OPEN;
        intake.last_target = Intake.Position.OPEN;

        TrajectorySequence preload = drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                .lineToLinearHeading(new Pose2d(30, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(48, 0, Math.toRadians(40)))
                .lineToLinearHeading(new Pose2d(53, 4, Math.toRadians(40)))
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(50, -24, Math.toRadians(90)), Math.toRadians(250))
                .build();

        TrajectorySequence intake = drive.trajectorySequenceBuilder(preload.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(50,-24,Math.toRadians(90)), Math.toRadians(250)).build();





        while(!isStopRequested()){

            snsr_loop += 1;
            if(snsr_loop > loop_max){
                snsr_loop = 0;
            }

            lift.time_elapsed = timer.time() - last_time;


            switch(machine){

                case INTAKE_UP:

                    lift.setTarget(Lift.lift_hold);
                    intake.target = Intake.Position.OPEN;
                    extension.setTarget(Extension.intake_pos);

                    act = true;

                    if(lift.state == Lift.State.HOLDING && intake.state == Intake.State.OPEN){
                        if(sensors.intakeReady){

                            lift.setTarget(1);
                            machine = FSM.INTAKE_DOWN;
                        }
                    }

                    break;

                case INTAKE_DOWN:

                    if(intake.state == Intake.State.CLOSED && intake.switch_cooldown){
                        lift.setTarget(Lift.lift_stack);
                        act = false;
                        machine = FSM.TRANSFER;
                    }

                    break;


                case TRANSFER:

                    if(gamepad1.y){
                        lift.setTarget(Lift.lift_low);
                        machine = FSM.RAISE;
                    }
                    if(gamepad1.x){
                        lift.setTarget(Lift.lift_mid);
                        machine = FSM.RAISE;
                    }

                    break;


                case RAISE:

                    if(lift.safeToExtend){
                        extension.setTarget(Extension.full_ext);
                    }

                    if(lift.state == Lift.State.HOLDING && extension.state == Extension.State.EXTENDED_MAX){
                        intake.target = Intake.Position.OPEN;
                        machine = FSM.RETURN;
                    }

                    break;

                case RETURN:

                    if(intake.state == Intake.State.OPEN){
                        extension.setTarget(Extension.intake_pos);
                    }
                    if(extension.state == Extension.State.MOVING && extension.safeToLower){
                        lift.setTarget(Lift.lift_hold);
                        machine = FSM.INTAKE_UP;
                    }
                    break;
            }

            intake.update(intaketime);
            intake.intake.setPosition(intake.target_pos);
            drive.update();
            extension.update(lift.time_elapsed);
            extension.extension.setPower(extension.out);
            sensors.update(act, snsr_loop, loop_max);
            lift.updateFeedforwardNew();
            lift.liftl.setPower(lift.outL);
            lift.liftr.setPower(lift.outL);
            last_time = timer.time();


        }

    }
}*/
