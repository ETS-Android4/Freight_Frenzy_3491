package org.firstinspires.ftc.teamcode.Competition.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Season_Setup.Ducky;

@TeleOp(name="Competition TeleOp", group="Competition")

public class DuckyTeleOp extends OpMode {

    // Initializing Robot Class
    Ducky ducky = new Ducky();


    /**
     * Initializing the Program
     */
    @Override
    public void init() {

        // Initialize all motors/ servos
        ducky.init(hardwareMap, telemetry);

        // Indicate that the program is running
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Alliance", Ducky.alliance);
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


        //------------------------------------------------------------------------------------------
        // Driving Controls
        //------------------------------------------------------------------------------------------

        // Reverse Drive
        if (gamepad1.left_trigger > 0) {

            // Telemetry Update
            telemetry.addData("Drive Mode: ", "Reverse");

            // Slow Speed
            if (gamepad1.right_trigger > 0) {
                ducky.FrontLeft.setPower(rightPower/-2);
                ducky.BackLeft.setPower((rightPower/-2)* Ducky.BACK_WHEEL_POWER_REDUCTION);
                ducky.FrontRight.setPower(leftPower/-2);
                ducky.BackRight.setPower((leftPower/-2)* Ducky.BACK_WHEEL_POWER_REDUCTION);

                // Telemetry Update
                telemetry.addData("Speed Mode: ", "Slow");

            // Normal Speed
            } else {
                ducky.FrontLeft.setPower(-rightPower);
                ducky.BackLeft.setPower((-rightPower)* Ducky.BACK_WHEEL_POWER_REDUCTION);
                ducky.FrontRight.setPower(-leftPower);
                ducky.BackRight.setPower((-leftPower)* Ducky.BACK_WHEEL_POWER_REDUCTION);

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
                ducky.BackLeft.setPower((leftPower/2)* Ducky.BACK_WHEEL_POWER_REDUCTION);
                ducky.FrontRight.setPower(rightPower/2);
                ducky.BackRight.setPower((rightPower/2)* Ducky.BACK_WHEEL_POWER_REDUCTION);

                // Telemetry Update
                telemetry.addData("Speed Mode: ", "Slow");

            // Normal Speed
            } else {
                ducky.FrontLeft.setPower(leftPower);
                ducky.BackLeft.setPower((leftPower)* Ducky.BACK_WHEEL_POWER_REDUCTION);
                ducky.FrontRight.setPower(rightPower);
                ducky.BackRight.setPower((rightPower)* Ducky.BACK_WHEEL_POWER_REDUCTION);

                // Telemetry Update
                telemetry.addData("Speed Mode: ", "Normal");
            }
        }


        //------------------------------------------------------------------------------------------
        // Mechanisms
        //------------------------------------------------------------------------------------------

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
        if (gamepad2.left_stick_y == 0) {
            if (gamepad2.dpad_up) {
                ducky.ArmCollecting();
            } else if (gamepad2.dpad_left) {
                ducky.ArmTopLevel();
            } else if (gamepad2.dpad_right) {
                ducky.ArmMidLevel();
            } else if (gamepad2.dpad_down) {
                ducky.ArmBottomLevel();
            }
        } else {
            ducky.RotateArm(gamepad2.left_stick_y);
        }


        // Carousel Spinner
        if (Ducky.alliance.equals("Blue")) {
            if (gamepad2.left_trigger > 0) {
                ducky.CarouselSpinnerBlue();
            } else if (gamepad2.left_bumper) {
                ducky.CarouselSpinnerOff();
            }
        } else if (Ducky.alliance.equals("Red"))  {
            if (gamepad2.left_trigger > 0) {
                ducky.CarouselSpinnerRed();
            } else if (gamepad2.left_bumper) {
                ducky.CarouselSpinnerOff();
            }
        }


        // Telemetry Update
        telemetry.addData("Left Side Power", leftPower);
        telemetry.addData("Right Side Power", rightPower);

        telemetry.addData("Arm Target Position",
                Ducky.ARM_COLLECTING_ENCODER_PULSES);
        telemetry.addData("Arm Encoder Pulses",
                ducky.ArmRotator.getCurrentPosition());

        telemetry.update();
    }

    /**
     * Code to run after Driver hits "Stop."
     */
    public void stop() {
    }
}
