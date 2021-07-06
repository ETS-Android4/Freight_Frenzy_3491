package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Robots.Beyonce;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;

@Disabled

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
        colorSensorL.enableLed(false);
        sleep(100);
        colorSensorL.enableLed(true);

        //beyonce.init();

        telemetry.addData("status", "init");

        beyonce.RingPusherRetract();
        //beyonce.GrabberUp();

        waitForStart();

        while (opModeIsActive()) {


            beyonce.ShooterOn();
            sleep(5000);

            beyonce.Shoot();
            beyonce.Shoot();
            beyonce.Shoot();

            sleep(50);
            beyonce.ShooterOff();

            beyonce.StrafeLeft(0.2);
            telemetry.addData("staus", "left");
            sleep(1000);
            beyonce.Stop();

            sleep(200);

            beyonce.DriveForward(0.1);
            telemetry.addData("staus", "forward");
            sleep(800);
            beyonce.Stop();

            sleep(300);

            clearTimer(1);

            while (opModeIsActive() && 38 > colorSensorL.red() && getMilliSeconds(1) < 3500) {
                telemetry.addData("status", getMilliSeconds(1));
                telemetry.addData("red", colorSensorL.red());

/*nice*/        beyonce.StrafeRight(0.05);
            }
            telemetry.addData("red", colorSensorL.red());

            beyonce.Stop();

            sleep(500);

            beyonce.StrafeLeft(0.1);
            sleep(1250);
            beyonce.Stop();

            sleep(200);

            beyonce.DriveBackward(0.1);
            sleep(1200);
            beyonce.Stop();

            sleep(300);

            //beyonce.GrabberDown();

            sleep(600);

            beyonce.DriveForward(0.3);
            sleep(300);
            beyonce.Stop();

        }


    }
}
