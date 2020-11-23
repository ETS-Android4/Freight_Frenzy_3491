package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Robots.Beyonce;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;

@Autonomous
public class ShootWobbleGoal extends AutoOpMode {
    @Override
    public void runOp() throws InterruptedException {
        Beyonce beyonce = new Beyonce();

        ColorSensor colorSensorL;
        colorSensorL = (ColorSensor) hardwareMap.get("ColourSensorL");

        ColorSensor colorSensorR;
        colorSensorR = (ColorSensor) hardwareMap.get("ColourSensorR");

        boolean red = opModeIsActive() && (30 < colorSensorL.red()) && (colorSensorL.red() < 38);
        boolean white = opModeIsActive() && 38 <= colorSensorL.red();


        //beyonce.init();

        telemetry.addData("status", "init");

        beyonce.RingPusherRetract();
        beyonce.GrabberUp();

        waitForStart();

        beyonce.ShooterOn();
        sleep(5000);

        beyonce.Shoot();
        beyonce.Shoot();
        beyonce.Shoot();

        sleep(50);
        beyonce.ShooterOff();

//        beyonce.StrafeRight(0.2);
//        sleep(400);
//        beyonce.Stop();

        beyonce.StrafeLeft(0.2);
        sleep(1000);
        beyonce.Stop();

        sleep(500);

        beyonce.DriveForward(0.3);
        sleep(200);
        beyonce.Stop();

        sleep(200);

        beyonce.StrafeLeft(0.2);
        sleep(100);
        beyonce.Stop();

        sleep(200);


        beyonce.StrafeRight(0.3);
        sleep(1000);

        sleep(200);



        while (white){
            beyonce.StrafeRight(0.2);
        }
        beyonce.Stop();

        beyonce.DriveBackward(0.2);
        sleep(1000);
        beyonce.Stop();

        beyonce.GrabberDown();

        beyonce.DriveForward(0.3);
        sleep(200);
        beyonce.Stop();




    }
}
