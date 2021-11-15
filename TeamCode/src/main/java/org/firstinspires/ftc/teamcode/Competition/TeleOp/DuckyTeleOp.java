package org.firstinspires.ftc.teamcode.Competition.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Season_Setup.Ducky;

@TeleOp
public class DuckyTeleOp extends OpMode {

    // Initializing Robot Class
    Ducky ducky = new Ducky();

    // Variable Declaration
    // N/A

    /**
     * Initializing the Program
     */
    @Override
    public void init() {

        // Initialize all motors/ servos
        ducky.init(hardwareMap, telemetry);

        // Indicate that the program is running
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /**
     * TeleOp loop once Driver hits "Play."
     */
    @Override
    public void loop() {

        // Initializing Joystick Control
        float leftPower = -gamepad1.left_stick_y;
        float rightPower = -gamepad1.right_stick_y;

        /* Driving Controls */
        // Reverse Drive
        if (gamepad1.left_trigger > 0) {

            // Telemetry Update
            telemetry.addData("Drive Mode: ", "Reverse");

            // Slow Speed
            if (gamepad1.right_trigger > 0) {
                ducky.FrontLeft.setPower(rightPower/-2);
                ducky.BackLeft.setPower((rightPower/-2)/ Ducky.BACK_WHEEL_POWER_REDUCTION);
                ducky.FrontRight.setPower(leftPower/-2);
                ducky.BackRight.setPower((leftPower/-2)/ Ducky.BACK_WHEEL_POWER_REDUCTION);

                // Telemetry Update
                telemetry.addData("Speed Mode: ", "Slow");

            // Normal Speed
            } else {
                ducky.FrontLeft.setPower(-rightPower);
                ducky.BackLeft.setPower((-rightPower)/ Ducky.BACK_WHEEL_POWER_REDUCTION);
                ducky.FrontRight.setPower(-leftPower);
                ducky.BackRight.setPower((-leftPower)/ Ducky.BACK_WHEEL_POWER_REDUCTION);

                // Telemetry Update
                telemetry.addData("Speed Mode: ", "Normal");
            }

        // Forward Drive
        } else {

            // Telemetry Update
            telemetry.addData("Drive Mode: ", "Forward");

            // Slow Speed
            if (gamepad1.right_trigger > 0) {
                ducky.FrontLeft.setPower(leftPower/2);
                ducky.BackLeft.setPower((leftPower/2)/ Ducky.BACK_WHEEL_POWER_REDUCTION);
                ducky.FrontRight.setPower(rightPower/2);
                ducky.BackRight.setPower((rightPower/2)/ Ducky.BACK_WHEEL_POWER_REDUCTION);

                // Telemetry Update
                telemetry.addData("Speed Mode: ", "Slow");

            // Normal Speed
            } else {
                ducky.FrontLeft.setPower(leftPower);
                ducky.BackLeft.setPower((leftPower)/ Ducky.BACK_WHEEL_POWER_REDUCTION);
                ducky.FrontRight.setPower(rightPower);
                ducky.BackRight.setPower((rightPower)/ Ducky.BACK_WHEEL_POWER_REDUCTION);

                // Telemetry Update
                telemetry.addData("Speed Mode: ", "Normal");
            }
        }

        /* Mechanisms */

        // Collector
        if (gamepad2.right_trigger > 0) {
            ducky.CollectorOn();
        }
        if (gamepad2.right_bumper) {
            ducky.CollectorReverse();
        }
        if (gamepad2.a) {
            ducky.CollectorOff();
        }

        // Arm Rotator
        ducky.RotateArm(gamepad2.left_stick_y/3);

//        if (gamepad2.dpad_up) {
//            ducky.ArmCollecting();
//        }
//        if (gamepad2.dpad_left) {
//            ducky.ArmTopLevel();
//        }
//        if (gamepad2.dpad_right) {
//            ducky.ArmMidLevel();
//        }
//        if (gamepad2.dpad_down) {
//            ducky.ArmBottomLevel();
//        }


        // Carousel Spinner
        if (gamepad2.left_trigger > 0) {
            ducky.CarouselSpinnerOn();
        }
        if(gamepad2.left_bumper) {
            ducky.CarouselSpinnerReverse();
        }
        if (gamepad2.b) {
            ducky.CarouselSpinnerOff();
        }

        // Telemetry Update
        telemetry.addData("Left Side Power", leftPower);
        telemetry.addData("Right Side Power", rightPower);

        telemetry.addData("left motor", ducky.BackLeft.getCurrentPosition());
        telemetry.addData("right motor", ducky.BackRight.getCurrentPosition());
        telemetry.update();
    }

    /**
     * Code to run after Driver hits "Stop."
     */
    public void stop() {
    }
}
