package org.firstinspires.ftc.teamcode.Competition.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Season_Setup.Ducky;


@Autonomous(name = "DuckWarehouse - P2, Red", group="Competition - Red")

public class DuckWarehouse_P2_Red extends LinearOpMode{

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
                ducky.backLeft.getCurrentPosition(),
                ducky.backRight.getCurrentPosition());
        telemetry.update();

        // Setting Alliance Colour for TeleOp
        ducky.writeAndRead("Red");

        // Telemetry Update
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.addData("Alliance", Ducky.alliance);
        telemetry.update();

        // Waiting for the program to start
        waitForStart();

        // Autonomous Pathing
        ducky.driveBackward_Encoder(10, 0.5, 5000);
        ducky.carouselSpinnerRed();
        Thread.sleep(4000);
        ducky.carouselSpinnerOff();

        ducky.driveForward_Encoder(15,0.5,3000);
        ducky.armMidLevel();
        ducky.driveForward_Encoder_WallRunners(61,0.5,5000);

        ducky.stop_Power();
    }
}
