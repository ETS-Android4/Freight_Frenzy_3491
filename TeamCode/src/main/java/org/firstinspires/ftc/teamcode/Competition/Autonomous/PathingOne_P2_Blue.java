package org.firstinspires.ftc.teamcode.Competition.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.Ducky;


@Autonomous(name="PathingOne - P2, Blue")

public class PathingOne_P2_Blue extends LinearOpMode {

    // Initializing Robot Class
    Ducky ducky = new Ducky();

    @Override
    public void runOpMode() {

        // Initialize all motors/ servos
        ducky.init(hardwareMap);

        // Reset Encoders, and Telemetry Update
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        ducky.FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ducky.BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ducky.FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ducky.BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Encoder Position Update
        telemetry.addData("Encoder Position",  "Starting Encoder Position",
                ducky.FrontLeft.getCurrentPosition(),
                ducky.BackLeft.getCurrentPosition(),
                ducky.FrontRight.getCurrentPosition(),
                ducky.BackRight.getCurrentPosition());
        telemetry.update();

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Waiting for the program to start
        waitForStart();

        // Autonomous Pathing
        ducky.DriveForward_Encoder(2,1);

    }
}
