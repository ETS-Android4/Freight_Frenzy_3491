package org.firstinspires.ftc.teamcode.Competition.z_SetUp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.Ducky;

@TeleOp
public class Turning_EncoderValueFinder extends OpMode {

    // Initializing Robot Class
    Ducky ducky = new Ducky();

    /**
     * Initializing the Program
     */
    @Override
    public void init() {

        // Initialize all motors/ servos
        ducky.init(hardwareMap);

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
        float turnLeft = -gamepad1.left_stick_y;
        float turnRight = -gamepad1.right_stick_y;

        /* Driving Controls */
        // Turning left
        if (-gamepad1.left_stick_y != 0) {

            // Telemetry Update
            telemetry.addData("Drive Mode: ", "Turning Left");

            // Slow Speed (speed/4)
            if (gamepad1.right_trigger > 0) {
                ducky.FrontLeft.setPower(-turnLeft/-4);
                ducky.BackLeft.setPower((-turnLeft/-4)/ 0.7559);
                ducky.FrontRight.setPower(turnLeft/-4);
                ducky.BackRight.setPower((turnLeft/-4)/ 0.7559);

                // Telemetry Update
                telemetry.addData("Speed Mode: ", "Slow (speed/4)");

                // Slow Speed (speed/2)
            } else {
                ducky.FrontLeft.setPower(-turnLeft/2);
                ducky.BackLeft.setPower((-turnLeft/2)/ 0.7559);
                ducky.FrontRight.setPower(turnLeft/2);
                ducky.BackRight.setPower((turnLeft/2)/ 0.7559);

                // Telemetry Update
                telemetry.addData("Speed Mode: ", "Slow (speed/2)");
            }

        // Turning Right
        } else if (-gamepad1.right_stick_y != 0) {

            // Telemetry Update
            telemetry.addData("Drive Mode: ", "Turning Right");

            // Slow Speed (speed/4)
            if (gamepad1.right_trigger > 0) {
                ducky.FrontLeft.setPower(turnRight/-4);
                ducky.BackLeft.setPower((turnRight/-4)/ 0.7559);
                ducky.FrontRight.setPower(-turnRight/-4);
                ducky.BackRight.setPower((-turnRight/-4)/ 0.7559);

                // Telemetry Update
                telemetry.addData("Speed Mode: ", "Slow (speed/4)");

                // Slow Speed (speed/2)
            } else {
                ducky.FrontLeft.setPower(turnRight/2);
                ducky.BackLeft.setPower((turnRight/2)/ 0.7559);
                ducky.FrontRight.setPower(-turnRight/2);
                ducky.BackRight.setPower((-turnRight/2)/ 0.7559);

                // Telemetry Update
                telemetry.addData("Speed Mode: ", "Slow (speed/2)");
            }
        }

        if (gamepad1.a) {
            ducky.BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ducky.BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }


        // Telemetry Update
        telemetry.addData("BackLeft Encoder Pulses",   ducky.BackLeft.getCurrentPosition());
        telemetry.addData("BackRight Encoder Pulses",  ducky.BackRight.getCurrentPosition());
        telemetry.update();
    }

    /**
     * Code to run after Driver hits "Stop."
     */
    @Override
    public void stop() {
    }
}
