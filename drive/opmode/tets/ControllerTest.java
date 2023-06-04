package org.firstinspires.ftc.teamcode.drive.opmode.tets;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.GorillabotsCentral;

@TeleOp(group = "drive")

public class ControllerTest extends GorillabotsCentral {
    @Override
    public void runOpMode() throws InterruptedException {


        initializeComponents();

        while(!isStopRequested()){


            telemetry.addData("Gamepad 1 Right Stick X: ", gamepad1.right_stick_x);
            telemetry.addData("Gamepad 1 Right Stick Y: ", gamepad1.right_stick_y);
            telemetry.addData("Gamepad 1 Left Stick X: ", gamepad1.left_stick_x);
            telemetry.addData("Gamepad 1 Left Stick Y: ", gamepad1.left_stick_y);
            telemetry.update();
        }

    }
}
