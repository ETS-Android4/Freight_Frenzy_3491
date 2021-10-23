package org.firstinspires.ftc.teamcode.Competition.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Ducky;
import org.firstinspires.ftc.teamcode.opMode_Support.AutoOpMode;


@Autonomous(name="PathingOne - P2, Blue")

public class PathingOne_P2_Blue extends AutoOpMode {

    // Initializing Robot Class
    Ducky ducky;

    @Override
    public void runOp() throws InterruptedException {

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Waiting for the program to start
        waitForStart();

        // Autonomous Pathing
        ducky.DriveForward(1);
        sleep(1000);
        ducky.TurnLeft(1);
        sleep(400);
        ducky.DriveForward(1);
        sleep(5000);
        ducky.Stop();
    }
}
