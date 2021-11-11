package org.firstinspires.ftc.teamcode.Test.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Season_Setup.Ducky;


@Autonomous(name="Encoder Only - Straight")

public class Encoder_Only_Test extends LinearOpMode {

    // Initializing Robot Class
    Ducky ducky = new Ducky();

    @Override
    public void runOpMode() {

        // Initialize all motors/ servos
        ducky.init(hardwareMap, telemetry);

        // Reset Encoders, and Telemetry Update
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        ducky.BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ducky.BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Encoder Position Update
        telemetry.addData("Encoder Position",  "Starting Encoder Position",
                ducky.BackLeft.getCurrentPosition(),
                ducky.BackRight.getCurrentPosition());
        telemetry.update();

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Waiting for the program to start
        waitForStart();

        // Autonomous Pathing
        int Cart_Wheel_Distance = 68;

        ducky.BackLeft.setTargetPosition(Cart_Wheel_Distance);
        ducky.BackRight.setTargetPosition(Cart_Wheel_Distance);

        ducky.BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ducky.BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ducky.FrontLeft.setPower(-1);
        ducky.BackLeft.setPower(-1/ 0.7559);
        ducky.FrontRight.setPower(-1);
        ducky.BackRight.setPower(-1/ 0.7559);

        while (ducky.BackLeft.isBusy() || ducky.BackRight.isBusy()) {
            telemetry.addData("Driving Forward, Encoder Pulses Left (Left Wheel)",
                    Cart_Wheel_Distance-ducky.BackLeft.getCurrentPosition());
            telemetry.addData("Driving Forward, Encoder Pulses Left (Right Wheel)",
                    Cart_Wheel_Distance-ducky.BackRight.getCurrentPosition());
            telemetry.update();
        }

        ducky.FrontLeft.setPower(0);
        ducky.BackLeft.setPower(0);
        ducky.FrontRight.setPower(0);
        ducky.BackRight.setPower(0);

        telemetry.addData("Encoder Position",  "Starting Encoder Position",
                ducky.BackLeft.getCurrentPosition(),
                ducky.BackRight.getCurrentPosition());
        telemetry.update();

//        ducky.DriveForward_Encoder(4,0.5);

    }
}
