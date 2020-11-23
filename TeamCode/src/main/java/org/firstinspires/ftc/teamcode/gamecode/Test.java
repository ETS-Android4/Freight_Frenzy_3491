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
        colorSensorL = (ColorSensor) hardwareMap.get("ColourSensorL");
        waitForStart();

        while (opModeIsActive()) {

        telemetry.addData("red", colorSensorL.red());
        //telemetry.addData("blue", colorSensorL.blue());
      //  telemetry.addData("green", colorSensorL.green());
        telemetry.addData("light", colorSensorL.alpha());
       // telemetry.addData("4 colour channels", colorSensorL.argb());
            }


        //red in between like 30 and 36

    }


}