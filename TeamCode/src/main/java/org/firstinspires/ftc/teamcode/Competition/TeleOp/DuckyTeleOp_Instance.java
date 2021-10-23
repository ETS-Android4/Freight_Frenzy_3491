package org.firstinspires.ftc.teamcode.Competition.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Ducky;
import org.firstinspires.ftc.teamcode.opMode_Support.TeleOpMode;

@TeleOp
public class DuckyTeleOp_Instance extends TeleOpMode {

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
        // telemetry.addData("Run Time", runtime.toString());
        telemetry.addData("Slow Button Enabled", slowButton);
        telemetry.addData("Left Side Power", leftPower);
        telemetry.addData("Right Side Power", rightPower);
        telemetry.update();
    }
}
