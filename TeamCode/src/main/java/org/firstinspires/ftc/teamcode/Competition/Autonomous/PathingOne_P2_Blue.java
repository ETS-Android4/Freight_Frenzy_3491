package org.firstinspires.ftc.teamcode.Competition.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Season_Setup.Ducky;
import org.firstinspires.ftc.teamcode.Season_Setup.Freight_Frenzy_Pipeline;


@Autonomous(name="PathingOne - P2, Blue", group="Competition - Blue")

public class PathingOne_P2_Blue extends LinearOpMode {

    // Initializing Robot Class
    Ducky ducky = new Ducky();

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize all motors/ servos
        ducky.init(hardwareMap, telemetry);
        Freight_Frenzy_Pipeline freight_frenzy_pipeline = new Freight_Frenzy_Pipeline();

        // Reset Encoders, and Telemetry Update
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

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
        ducky.DriveBackward_Encoder(1,-0.3);
        Thread.sleep(1000);
        ducky.turn_P(-45,4000);

        if (Freight_Frenzy_Pipeline.positionOfTeamShippingElement == Freight_Frenzy_Pipeline.Pipeline.ElementPosition.LEFT) {
            ducky.ArmBottomLevel();
        } else if (Freight_Frenzy_Pipeline.positionOfTeamShippingElement == Freight_Frenzy_Pipeline.Pipeline.ElementPosition.CENTER) {
            ducky.ArmMidLevel();
        } else if (Freight_Frenzy_Pipeline.positionOfTeamShippingElement == Freight_Frenzy_Pipeline.Pipeline.ElementPosition.RIGHT) {
            ducky.ArmTopLevel();
        }
        ducky.DriveBackward_Encoder(2,0.5);
        ducky.CollectorReverse();
        Thread.sleep(1000);
        ducky.CollectorOff();
        ducky.ArmCollecting();
        ducky.turn_P(-45,4000);
        ducky.ArmRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ducky.DriveForward_Encoder(20,0.5);
        ducky.turn_P(-90,4000);
        ducky.DriveBackward_Encoder(10,0.5);
        ducky.CarouselSpinnerOn();
        Thread.sleep(4000);
        ducky.turn_P(-90,4000);
        ducky.DriveForward_Encoder(144,1);
    }
}
