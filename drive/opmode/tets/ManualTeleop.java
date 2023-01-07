package org.firstinspires.ftc.teamcode.drive.opmode.tets;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.ExtentionMAG;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Intake;

@Disabled
@TeleOp(group = "drive")
public class ManualTeleop extends GorillabotsCentral {
    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        ExtentionMAG extentionMAG = new ExtentionMAG(hardwareMap);

        double drive_speed_constant = 0.8;

        double last_time = 0;

        ElapsedTime intaketimer = new ElapsedTime();
        ElapsedTime exttimer = new ElapsedTime();

        waitForStart();

        while(!isStopRequested()){


            lift.time_elapsed = timer.time() - last_time;

            if (gamepad2.right_trigger < 0.15 && gamepad2.left_trigger < 0.15) {
                lift.liftr.setPower(lift.Rf);
                lift.liftl.setPower(lift.Lf);
            }

            if (gamepad2.right_trigger > 0.15 && gamepad2.left_trigger < 0.15) {
                lift.liftl.setPower(gamepad2.right_trigger + lift.Lf);
                lift.liftr.setPower(gamepad2.right_trigger + lift.Rf);
            }

            if (gamepad2.left_trigger > 0.15 && gamepad2.right_trigger < 0.15) {
                lift.liftr.setPower(-gamepad2.left_trigger * 0.6 + lift.Rf);
                lift.liftl.setPower(-gamepad2.left_trigger * 0.6 + lift.Lf);
            }

            if (gamepad2.right_trigger > 0.15 && gamepad2.left_trigger > 0.15) {
                lift.liftr.setPower(lift.Rf);
                lift.liftl.setPower(lift.Lf);
            }

            if(gamepad2.a){
                intake.target = Intake.Position.OPEN;
                intake.intake.setPosition(intake.OPEN);
            }
            if(gamepad2.b){
                intake.target = Intake.Position.OPEN;
                intake.intake.setPosition(intake.CLOSED);
            }

            if(gamepad2.left_bumper){
                extentionMAG.setTarget(ExtentionMAG.State.RETRACTED);
            }

            if(gamepad2.right_bumper){
                extentionMAG.setTarget(ExtentionMAG.State.EXTENDED);
            }

            if(gamepad1.left_bumper){
                flipper.flipper.setPosition(flipper.RAISED);
            }
            if(gamepad1.right_bumper){
                flipper.flipper.setPosition(flipper.LOWERED);
            }

            drive_speed_constant = (1 - ((lift.getPositionL() + 950) / 3100)) + 0.25;

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * drive_speed_constant,
                            -gamepad1.left_stick_x * drive_speed_constant,
                            -gamepad1.right_stick_x * drive_speed_constant
                    )
            );

            drive.update();
            //lift.updateFeedforwardNew();
            extentionMAG.update(exttimer);
            intake.update(intaketimer);
            last_time = timer.time();
        }
    }
}
