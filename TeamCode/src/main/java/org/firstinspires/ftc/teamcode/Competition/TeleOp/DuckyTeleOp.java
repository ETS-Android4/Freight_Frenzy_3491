package org.firstinspires.ftc.teamcode.Competition.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robots.Ducky;
import org.firstinspires.ftc.teamcode.opMode_Support.TeleOpMode;

@TeleOp
public class DuckyTeleOp extends TeleOpMode {

    // Initializing Robot Class
    Ducky ducky;

    // Variable Declaration
    boolean slowButton;

    /**
     * Initializing the Program
     */
    public void initialize() {

        // Indicate that the program is running
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void loopOpMode() {

        // Robot Driving
//        if (gamepad1.right_trigger > 0) {
//            if (gamepad1.left_trigger > 0) {
//                ducky.ducky.FrontLeft.setPower(1);
//                ducky.BackLeft.setPower((pivot + (vertical - horizontal)) / -3);
//                ducky.FrontRight.setPower((-pivot + (vertical - horizontal)) / -3);
//                ducky.BackRight.setPower((pivot + (vertical + horizontal)) / -3);
//                telemetry.addData("Drive Mode: ", "Reverse");
//
//            } else {
//                ducky.FrontLeft.setPower((pivot + (vertical + horizontal)) / 2);
//                ducky.BackLeft.setPower((-pivot + (vertical - horizontal)) / 2);
//                ducky.FrontRight.setPower((pivot + (vertical - horizontal)) / 2);
//                ducky.BackRight.setPower((-pivot + (vertical + horizontal)) / 2);
//                telemetry.addData("Drive Mode: ", "Forward");
//            }
//            telemetry.addData("Speed Mode: ", "slow");
//
//        } else {
//            if (gamepad1.left_trigger > 0) {
//                ducky.FrontLeft.setPower((-pivot + (vertical + horizontal)) * -1);
//                ducky.BackLeft.setPower((pivot + (vertical - horizontal)) * -1);
//                ducky.FrontRight.setPower((-pivot + (vertical - horizontal)) * -1);
//                ducky.BackRight.setPower((pivot + (vertical + horizontal)) * -1);
//                telemetry.addData("Drive Mode: ", "Reverse");
//
//            } else {
//                ducky.FrontLeft.setPower((pivot + (vertical + horizontal)));
//                ducky.BackLeft.setPower((-pivot + (vertical - horizontal)));
//                ducky.FrontRight.setPower((pivot + (vertical - horizontal)));
//                ducky.BackRight.setPower((-pivot + (vertical + horizontal)));
//                telemetry.addData("Drive Mode: ", "Forward");
//
//            }

        // Initializing Joystick Control

        float leftPower = -gamepad1.left_stick_y;
        float rightPower = -gamepad1.right_stick_y;

        //// Driving Controls
        // Slow Button
        if (gamepad1.left_trigger > 0) {
            ducky.FrontLeft.setPower(leftPower/2);
            ducky.BackLeft.setPower(rightPower/2);
            ducky.FrontRight.setPower(leftPower/2);
            ducky.BackRight.setPower(rightPower/2);

            slowButton = true;

            //Normal Driving
        } else {
            ducky.FrontLeft.setPower(leftPower);
            ducky.BackLeft.setPower(rightPower);
            ducky.FrontRight.setPower(leftPower);
            ducky.BackRight.setPower(rightPower);

            slowButton = false;
        }

        telemetry.addData("Speed Mode:", "fast");

    // Keeps user updated
    // Telemetry Update
    //elemetry.addData("Run Time", runtime.toString());
    telemetry.addData("Slow Button Enabled", slowButton);
    telemetry.addData("Left Side Power", leftPower);
    telemetry.addData("Right Side Power", rightPower);
    telemetry.update();
    }
}

