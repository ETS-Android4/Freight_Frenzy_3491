package org.firstinspires.ftc.teamcode.Competition.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.Ducky;
import org.firstinspires.ftc.teamcode.opMode_Support.TeleOpMode;

@TeleOp
public class DuckyTeleOp extends TeleOpMode {

    // Initializing Robot Class
    Ducky ducky = new Ducky();

    // Variable Declaration
    // N/A

    /**
     * Initializing the Program
     */
    public void initialize() {

        // Initialize all motors/ servos
        ducky.init(hardwareMap);

//        /* Uncomment if program crashes */
//        // Setting Motors to run with Encoders
//        ducky.FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        ducky.BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        ducky.FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        ducky.BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Indicate that the program is running
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void loopOpMode() {

        // Initializing Joystick Control
        float leftPower = -gamepad1.left_stick_y;
        float rightPower = -gamepad1.right_stick_y;

        /* Driving Controls */
        // Reverse Drive
        if (gamepad1.right_trigger > 0) {

            // Telemetry Update
            telemetry.addData("Drive Mode: ", "Reverse");

            // Slow Speed
            if (gamepad1.left_trigger > 0) {
                ducky.FrontLeft.setPower(leftPower/-2);
                ducky.BackLeft.setPower((leftPower/-2)/ 0.7559);
                ducky.FrontRight.setPower(rightPower/-2);
                ducky.BackRight.setPower((rightPower/-2)/ 0.7559);

                // Telemetry Update
                telemetry.addData("Speed Mode: ", "Slow");

            // Normal Speed
            } else {
                ducky.FrontLeft.setPower(-leftPower);
                ducky.BackLeft.setPower((-leftPower)/ 0.7559);
                ducky.FrontRight.setPower(-rightPower);
                ducky.BackRight.setPower((-rightPower)/ 0.7559);

                // Telemetry Update
                telemetry.addData("Speed Mode: ", "Normal");
            }

        // Forward Drive
        } else {

            // Telemetry Update
            telemetry.addData("Drive Mode: ", "Forward");

            // Slow Speed
            if (gamepad1.left_trigger > 0) {
                ducky.FrontLeft.setPower(leftPower/2);
                ducky.BackLeft.setPower((leftPower/2)/ 0.7559);
                ducky.FrontRight.setPower(rightPower/2);
                ducky.BackRight.setPower((rightPower/2)/ 0.7559);

                // Telemetry Update
                telemetry.addData("Speed Mode: ", "Slow");

            // Normal Speed
            } else {
                ducky.FrontLeft.setPower(leftPower);
                ducky.BackLeft.setPower((leftPower)/ 0.7559);
                ducky.FrontRight.setPower(rightPower);
                ducky.BackRight.setPower((rightPower)/ 0.7559);

                // Telemetry Update
                telemetry.addData("Speed Mode: ", "Normal");
            }
        }

        // Telemetry Update
        telemetry.addData("Left Side Power", leftPower);
        telemetry.addData("Right Side Power", rightPower);
        telemetry.update();
    }
}
