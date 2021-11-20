package org.firstinspires.ftc.teamcode.Competition.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Season_Setup.Ducky;
import org.firstinspires.ftc.teamcode.Season_Setup.Freight_Frenzy_Pipeline;


@Autonomous(name="HubDuckStorageUnit - P2, Blue", group="Competition - Blue")

public class HubDuckStorageUnit_P2_Blue extends LinearOpMode {

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

        //----------------------------------------------------------------------------------------------
        // Autonomous Pathing
        //----------------------------------------------------------------------------------------------

        // Detecting which autonomous state to run
        if (Freight_Frenzy_Pipeline.positionOfTeamShippingElement == Freight_Frenzy_Pipeline.Pipeline.ElementPosition.LEFT) {
            ducky.leftState = true;
            telemetry.addData("Team Shipping Element Position", "Left");
            telemetry.update();

        } else if (Freight_Frenzy_Pipeline.positionOfTeamShippingElement == Freight_Frenzy_Pipeline.Pipeline.ElementPosition.CENTER) {
            ducky.centerState = true;
            telemetry.addData("Team Shipping Element Position", "Center");
            telemetry.update();

        } else if (Freight_Frenzy_Pipeline.positionOfTeamShippingElement == Freight_Frenzy_Pipeline.Pipeline.ElementPosition.RIGHT) {
            ducky.rightState = true;
            telemetry.addData("Team Shipping Element Position", "Right");
            telemetry.update();

        }

        // Drive backwards
        ducky.DriveBackward_Encoder(10,-0.3,5000);

        // Move arm to position
        if (ducky.leftState) {
            ducky.ArmBottomLevel();
        } else if (ducky.centerState) {
            ducky.ArmMidLevel();
        } else if (ducky.rightState) {
            ducky.ArmTopLevel();
        }

        // Turn Towards Alliance Specific Shipping Hub
        ducky.turn_P(-90,3000, 1000);

        // Move closer to Shipping Hub and score
        ducky.DriveBackward_Encoder(2,0.5,5000);
        ducky.CollectorReverse();
        Thread.sleep(1000);
        ducky.CollectorOff();

        // Move arm back to collecting position
        ducky.ArmCollecting();

        // Turn and drive to carousel
        ducky.DriveForward_Encoder(20,0.3,3000);
        ducky.ArmRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ducky.turn_P(-90,4000, 1000);
        ducky.DriveBackward_Encoder(20,0.2,5000);

        // Spin Carousel
        ducky.CarouselSpinnerBlue();
        Thread.sleep(4000);
        ducky.CarouselSpinnerOff();

        // Drive into storage unit
        ducky.DriveForward_Encoder(7,0.5,5000);
    }
}
