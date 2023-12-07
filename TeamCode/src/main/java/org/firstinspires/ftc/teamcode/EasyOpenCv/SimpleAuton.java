package org.firstinspires.ftc.teamcode.EasyOpenCv;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


//!! Note: Does Not work, Pls fix and test this program before running (Ethan) !!
//todo: fix null pointer exception

@Autonomous(name="SimpleAuton")
public class SimpleAuton extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "left_front_drive");
    DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "left_rear_drive");
    DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "right_rear_drive");
    DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "right_front_drive");


    @Override
    public void runOpMode() throws InterruptedException {

        while (opModeIsActive()) {
            double y =  0;
            double x =  0;
            double rx = 0;

            if (runtime.seconds() < 1.0) {

                y = 0.3;

            }
//            else
//            {
//                y = 0;
//            }

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightBack.setPower(frontRightPower);
            rightFront.setPower(backRightPower);

        }
    }
}