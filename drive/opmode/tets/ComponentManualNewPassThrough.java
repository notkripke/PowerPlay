package org.firstinspires.ftc.teamcode.drive.opmode.tets;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.NewPassthrough;

@TeleOp(group = "drive")
public class ComponentManualNewPassThrough extends GorillabotsCentral {
    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        double pwr = 0;

        ElapsedTime pass_timer = new ElapsedTime();

        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        drive.setPoseEstimate(new Pose2d(0,0,0));

        passthrough.setTarget(NewPassthrough.State.RETRACTED);

        double closed = 0;
        double open = 0.2;

        intake.intake.setPosition(open);

        waitForStart();

        while(!isStopRequested()){
            if(gamepad1.a){
                intake.intake.setPosition(open);
            }
            if(gamepad1.b){
                intake.intake.setPosition(closed);
            }

            if (gamepad1.right_trigger < 0.15 && gamepad1.left_trigger < 0.15) {
                lift.liftr.setPower(lift.Rf);
                lift.liftl.setPower(lift.Lf);
            }

            if (gamepad1.right_trigger > 0.15 && gamepad1.left_trigger < 0.15) {
                lift.liftl.setPower(gamepad1.right_trigger * 0.80 + lift.Lf);
                lift.liftr.setPower(gamepad1.right_trigger * 0.80 + lift.Rf);
            }

            if (gamepad1.left_trigger > 0.15 && gamepad1.right_trigger < 0.15) {
                lift.liftr.setPower(-gamepad1.left_trigger * 0.4 + lift.Rf);
                lift.liftl.setPower(-gamepad1.left_trigger * 0.4 + lift.Lf);
            }

            if (gamepad1.right_trigger > 0.15 && gamepad1.left_trigger > 0.15) {
                lift.liftr.setPower(lift.Rf);
                lift.liftl.setPower(lift.Lf);
            }

            /*
            if(gamepad1.right_bumper){
                passthrough.servo.setPower(0.8);
            }
            if(gamepad1.left_bumper){
                passthrough.servo.setPower(-0.8);
            }
            if(!gamepad1.left_bumper && !gamepad1.right_bumper) {
                passthrough.servo.setPower(0);
            }
             */


            /*if (gamepad1.right_bumper) {
                passthrough.setTarget(NewPassthrough.State.EXTENDED);
            }
            if (gamepad1.left_bumper) {
                passthrough.setTarget(NewPassthrough.State.RETRACTED);
            }*/

            if(gamepad1.right_bumper && !gamepad1.left_bumper){
                pwr = -1;
            }

            if(gamepad1.left_bumper && !gamepad1.right_bumper){
                pwr = 1;
            }

            if(!gamepad1.right_bumper && !gamepad1.left_bumper){
                pwr = 0;
            }

            //passthrough.update(pass_timer);
            passthrough.servo.setPower(pwr);



            telemetry.addData("Front Switch", passthrough.getFrontState());
            telemetry.addData("Back Switch", passthrough.getBackState());
            telemetry.addData("intake position", intake.intake.getPosition());
            telemetry.addData("servo current power", passthrough.servo.getPower());
            telemetry.addData("pass state: ", passthrough.state);
            telemetry.addData("pass target: ", passthrough.target);
            telemetry.addData("out", pwr);
            telemetry.addData("closed", closed);
            telemetry.addData("open", open);
            telemetry.addData("touched: ", passthrough.touched);
            telemetry.addData("inc: ", passthrough.inc);
            telemetry.addData("done spring: ", passthrough.done_spring);
            telemetry.addData("timer thing: ", pass_timer.milliseconds());
            telemetry.update();

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y*.7,
                            -gamepad1.left_stick_x*.7,
                            -gamepad1.right_stick_x*.7
                    )
            );

            drive.update();

        }


    }
}