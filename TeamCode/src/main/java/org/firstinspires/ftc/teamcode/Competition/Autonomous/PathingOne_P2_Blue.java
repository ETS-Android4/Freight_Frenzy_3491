package org.firstinspires.ftc.teamcode.Competition.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Season_Setup.Ducky;


@Autonomous(name="PathingOne - P2, Blue", group="Competition - Blue")

public class PathingOne_P2_Blue extends LinearOpMode {

    // Initializing Robot Class
    Ducky ducky = new Ducky();

    @Override
    public void runOpMode() throws InterruptedException {

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
        ducky.DriveBackward_Encoder_IMU(10,-0.3);
        Thread.sleep(1000);
        ducky.TurnRight_IMU(45,0.3);
        Thread.sleep(1000);
        ducky.DriveBackward_Encoder_IMU(10,-0.3);
        Thread.sleep(1000);
        ducky.TurnRight_IMU(90,0.3);
        Thread.sleep(1000);
        ducky.DriveForward_Encoder_IMU(13,0.3);
        Thread.sleep(1000);
        ducky.CarouselSpinnerOn();
        Thread.sleep(3000);
        ducky.CarouselSpinnerOff();
    }
}
