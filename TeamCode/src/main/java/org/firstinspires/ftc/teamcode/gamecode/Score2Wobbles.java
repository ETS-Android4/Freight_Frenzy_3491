package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Robots.Beyonce;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;

@Disabled

@Autonomous
public class Score2Wobbles extends AutoOpMode {

    @Override
    public void runOp() throws InterruptedException {
        Beyonce beyonce = new Beyonce();
        ColorSensor colourSensor;
        colourSensor = (ColorSensor) hardwareMap.get("ColourSensorL");

        //beyonce.init();

        telemetry.addData("status", "init");



        waitForStart();

        //first wobble goal
//        beyonce.DriveBackward(0.5);
//        sleep(1000);
//        beyonce.StrafeRight(0.75);
//        sleep(4000);
//        beyonce.DriveForward(0.75);
//        sleep(300);
//        beyonce.Stop();
//        sleep(500);
//        beyonce.ArmDown(-0.5);
//        sleep(1800);
//        beyonce.ArmDown(-0.25);
//        sleep(250);
//        sleep(500);
//        beyonce.ClawOpen();
//        sleep(100);
//        beyonce.DriveForward(0.75);
//        sleep(1700);
//
//        //go back to drop zone wall
//        beyonce.StrafeLeft(1);
//        sleep(2500);
//        beyonce.StrafeLeft(0.3);
//        sleep(1000);

        //go to wobble goal and bring it to tower wall
        addTimer();
        while (colourSensor.red() < 27 /*&& getSeconds() < 10*/) {
            telemetry.addData("colour sensor value: ", colourSensor.red());
            telemetry.update();
            beyonce.DriveBackward(0.1);
        }
//        //pause after colour detected
//        beyonce.Stop();
//        sleep(500);
//        beyonce.DriveBackward(1);
//        sleep(500);
//        //pause again after driving backwards
//        beyonce.Stop();
//        sleep(500);
//
//        //put wobble goal in corner and go to park
//        beyonce.StrafeRight(1);
///*nice*/  sleep(2500);
//        beyonce.StrafeRight(0.3);
//        sleep(1000);
//        beyonce.DriveForward(1);
//        sleep(500);
//        beyonce.StrafeRight(0.75);
//        sleep(500);
//        beyonce.DriveBackward(1);
//        sleep(1000);
//
//        beyonce.Stop();



    }
}