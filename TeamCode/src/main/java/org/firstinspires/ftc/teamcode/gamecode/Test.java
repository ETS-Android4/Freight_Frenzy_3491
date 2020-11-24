package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robots.Beyonce;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;


@Autonomous
public class Test extends AutoOpMode {
    org.firstinspires.ftc.teamcode.Robots.Beyonce Beyonce;

    @Override
    public void runOp() throws InterruptedException {
        Beyonce beyonce = new Beyonce();
        ColorSensor colorSensorL;
        colorSensorL = (ColorSensor) hardwareMap.get("ColourSensorR");
        boolean white = opModeIsActive() && 38 > colorSensorL.red();

        beyonce.GrabberUp();

        waitForStart();

        beyonce.StrafeLeft(0.2);
        telemetry.addData("staus", "left");
        sleep(1000);
        beyonce.Stop();

        sleep(200);

        beyonce.DriveForward(0.1);
        telemetry.addData("staus", "forward");
        sleep(700);
        beyonce.Stop();

        sleep(300);

        clearTimer(1);

        while (opModeIsActive() && 38 > colorSensorL.red() && getSeconds(1) < 4){
            telemetry.addData("status", getSeconds(1));
            telemetry.addData("red", colorSensorL.red());

            beyonce.StrafeRight(0.1);
        }
        telemetry.addData("staus", "white");
        telemetry.addData("red", colorSensorL.red());

        beyonce.Stop();

        sleep(500);

        beyonce.StrafeLeft(0.1);
        sleep(1250);
        beyonce.Stop();

        sleep(200);

        beyonce.DriveBackward(0.1);
        sleep(1300);
        beyonce.Stop();

        sleep(300);

        beyonce.GrabberDown();

        sleep(600);

        beyonce.DriveForward(0.3);
        sleep(300);
        beyonce.Stop();



//
//        while (opModeIsActive()) {
//
//        telemetry.addData("red", colorSensorL.red());
//        //telemetry.addData("blue", colorSensorL.blue());
//      //  telemetry.addData("green", colorSensorL.green());
//        //telemetry.addData("light", colorSensorL.alpha());
//       // telemetry.addData("4 colour channels", colorSensorL.argb());
//            }


        //red in between like 30 and 36

    }


}