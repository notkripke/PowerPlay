package org.firstinspires.ftc.teamcode.drive.opmode.tets;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;
import org.firstinspires.ftc.teamcode.drive.opmode.Components.Intake;

public class ComponentTest extends GorillabotsCentral {
    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();

        while(!isStopRequested()){


            if(gamepad1.right_trigger > 0.1 && gamepad1.left_trigger < 0.1){
                lift.liftr.setPower(gamepad1.right_trigger);
                lift.liftl.setPower(gamepad1.left_trigger);
            }

            if(gamepad1.right_trigger < 0.1 && gamepad1.left_trigger > 0.1){
                lift.liftl.setPower(-gamepad1.left_trigger);
                lift.liftr.setPower(-gamepad1.left_trigger);
            }

            if(gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.1){
                lift.liftr.setPower(0);
                lift.liftl.setPower(0);
            }

            if(gamepad1.right_bumper && !gamepad1.left_bumper){
                extension.extension.setPower(0.5);
            }
            if(gamepad1.left_bumper && !gamepad1.right_bumper){
                extension.extension.setPower(-0.5);
            }
            if(!gamepad1.right_bumper && !gamepad1.left_bumper){
                extension.extension.setPower(0);
            }

            if(gamepad1.a){
                intake.target = Intake.Position.CLOSED;
            }
            if(gamepad1.b){
                intake.target = Intake.Position.OPEN;
            }
            intake.update();

            dashboardTelemetry.addData("Lift L: ", lift.getPositionL());
            dashboardTelemetry.addData("Lift R: ", lift.getPositionR());
            dashboardTelemetry.addData("Intake Target: ", intake.target);
            dashboardTelemetry.addData("Intake State: ", intake.state);
            dashboardTelemetry.addData("Intake Pos: ", intake.intake.getPosition());
            dashboardTelemetry.addData("Extention Pos: ", extension.getPosition());




        }

    }
}
