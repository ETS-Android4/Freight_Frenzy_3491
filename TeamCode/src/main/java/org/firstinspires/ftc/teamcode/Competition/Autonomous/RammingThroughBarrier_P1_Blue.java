package org.firstinspires.ftc.teamcode.Competition.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.Ducky;


@Autonomous(name="Ramming through Barrier - P1, Blue")

public class RammingThroughBarrier_P1_Blue extends LinearOpMode {

    // Initializing Robot Class
    Ducky ducky = new Ducky();

    @Override
    public void runOpMode() {

        // Initialize all motors/ servos
        ducky.init(hardwareMap);

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Waiting for the program to start
        waitForStart();

        // Autonomous Pathing
        ducky.DriveForward(1,1000);
        sleep(500);
        ducky.TurnLeft(1,400);
        sleep(500);
        ducky.DriveForward(1,2000);
    }
}
