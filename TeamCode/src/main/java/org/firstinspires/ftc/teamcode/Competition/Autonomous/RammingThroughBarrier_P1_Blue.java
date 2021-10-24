package org.firstinspires.ftc.teamcode.Competition.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robots.Ducky;
import org.firstinspires.ftc.teamcode.opMode_Support.AutoOpMode;


@Autonomous(name="Ramming through Barrier - P1, Blue")

public class RammingThroughBarrier_P1_Blue extends LinearOpMode {

    // Initializing Robot Class
    Ducky ducky = new Ducky();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        ducky.init(hardwareMap);


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
        sleep(2000);
        ducky.Stop();
    }
}
