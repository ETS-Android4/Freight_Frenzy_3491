package org.firstinspires.ftc.teamcode.z_Old_Programs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled

@TeleOp
public class Basic_DuckyTeleOp extends OpMode {

    // Declaring Motor Variables
    public DcMotor FrontLeft, BackLeft, FrontRight, BackRight;
    public boolean slowButton;

    // Declaring stuff
    public void init() {

        // Drive Motors
        FrontLeft = hardwareMap.dcMotor.get("frontL");
        BackLeft = hardwareMap.dcMotor.get("backL");
        FrontRight = hardwareMap.dcMotor.get("frontR");
        BackRight = hardwareMap.dcMotor.get("backR");

        // Setting Motor Direction
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {

        // Initializing Joystick Control

        float leftPower = -gamepad1.left_stick_y;
        float rightPower = -gamepad1.right_stick_y;

        //// Driving Controls
        // Slow Button
        if (gamepad1.left_trigger > 0) {
            FrontLeft.setPower(leftPower/2);
            BackLeft.setPower(rightPower/2);
            FrontRight.setPower(leftPower/2);
            BackRight.setPower(rightPower/2);

            slowButton = true;

            //Normal Driving
        } else {
            FrontLeft.setPower(leftPower);
            BackLeft.setPower(rightPower);
            FrontRight.setPower(leftPower);
            BackRight.setPower(rightPower);

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

    /**
     * Code to run after Driver hits "Stop."
     */
    public void stop() {
    }
}

