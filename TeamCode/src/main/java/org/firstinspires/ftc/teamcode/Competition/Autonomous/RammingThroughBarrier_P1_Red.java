package org.firstinspires.ftc.teamcode.Competition.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Season_Setup.Ducky;


@Autonomous(name="Ramming through Barrier - P1, Red", group="Competition - Blue")

public class RammingThroughBarrier_P1_Red extends LinearOpMode {

    // Initializing Robot Class
    Ducky ducky = new Ducky();

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize all motors/ servos
        ducky.init(hardwareMap, telemetry);

        // Reset Encoders, and Telemetry Update
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        // Encoder Position Update
        telemetry.addData("Encoder Position",  "Starting Encoder Position",
                ducky.BackLeft.getCurrentPosition(),
                ducky.BackRight.getCurrentPosition());
        telemetry.update();

        // Telemetry Update
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Waiting for the program to start
        waitForStart();

        // Autonomous Pathing
        ducky.DriveBackward_Encoder(5,0.5);
        ducky.turn_P(-90,4000);
        ducky.DriveForward_Power(1);
        Thread.sleep(2500);
        ducky.Stop_Power();
        Thread.sleep(1000);
        ducky.DriveBackward_Encoder(1,0.5);
        ducky.turn_P(90,4000);
        ducky.DriveForward_Encoder(1,0.5);
        ducky.turn_P(180,4000);
    }
}
