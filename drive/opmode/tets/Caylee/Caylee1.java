package org.firstinspires.ftc.teamcode.drive.opmode.tets.Caylee;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "drive")
public class Caylee1 extends LinearOpMode {

    public void runOpMode() throws InterruptedException {


        DcMotor duck;
        duck = hardwareMap.dcMotor.get("duck");

        double power = 0;
        double inrement = 0.0001;
        boolean good = true;

        waitForStart();

        while (!isStopRequested()) {

            if(power<1 && good) {
                duck.setPower(power);


                power += inrement;
            }

              if (power>=1) {
                  good = false;
              }


            if (!good) {
                duck.setPower(power);

                power = power - inrement;
            }

            telemetry.addData("Good", good);
            telemetry.addData("power",power);
            telemetry.update();
        }

    }
}
