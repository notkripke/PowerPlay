package org.firstinspires.ftc.teamcode.drive.opmode.tets;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.ExtentionMAG;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Intake;

@TeleOp(group = "drive")
public class ComponentTestExtMag extends GorillabotsCentral {
    @Override
    public void runOpMode() throws InterruptedException {

        initializeComponents();

        ExtentionMAG extentionmag = new ExtentionMAG(hardwareMap);

        drive.setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        drive.setPoseEstimate(new Pose2d(0,0,0));

        intake.intake.setPosition(intake.intake.getPosition());

        intake.testpos = intake.intake.getPosition();

        intake.target = Intake.Position.OPEN;

        waitForStart();

        while(!isStopRequested()){

            if(!gamepad2.right_bumper && !gamepad2.left_bumper){
                extentionmag.extension.setPower(0);
            }

            if(gamepad2.right_bumper && !gamepad2.left_bumper){
                extentionmag.extension.setPower(1);
            }

            if(!gamepad2.right_bumper && gamepad2.left_bumper){
                extentionmag.extension.setPower(-1);
            }

            if(gamepad2.a){
                intake.intake.setPosition(intake.OPEN);
            }
            if(gamepad2.b){
                intake.intake.setPosition(intake.CLOSED);
            }

            if(gamepad1.left_bumper){
                flipper.flipper.setPosition(flipper.RAISED);
            }
            if(gamepad1.right_bumper){
                flipper.flipper.setPosition(flipper.LOWERED);
            }

            if (gamepad2.right_trigger < 0.15 && gamepad2.left_trigger < 0.15) {
                lift.liftr.setPower(lift.Rf);
                lift.liftl.setPower(lift.Lf);
            }

            if (gamepad2.right_trigger > 0.15 && gamepad2.left_trigger < 0.15) {
                lift.liftl.setPower(gamepad2.right_trigger * 0.65 + lift.Lf);
                lift.liftr.setPower(gamepad2.right_trigger * 0.65 + lift.Rf);
            }

            if (gamepad2.left_trigger > 0.15 && gamepad2.right_trigger < 0.15) {
                lift.liftr.setPower(-gamepad2.left_trigger * 0.4 + lift.Rf);
                lift.liftl.setPower(-gamepad2.left_trigger * 0.4 + lift.Lf);
            }

            if (gamepad2.right_trigger > 0.15 && gamepad2.left_trigger > 0.15) {
                lift.liftr.setPower(lift.Rf);
                lift.liftl.setPower(lift.Lf);
            }

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
