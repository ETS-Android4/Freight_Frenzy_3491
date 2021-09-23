package org.firstinspires.ftc.teamcode.Test.TeleOp;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "ColorSensorDemo")
public class ColourSensorDemo extends LinearOpMode {

    private DcMotor FrontL;
    private DcMotor BackL;
    private DcMotor FrontR;
    private DcMotor BackR;
    private ColorSensor ColorSensorL;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int colorValue;

        FrontL = hardwareMap.get(DcMotor.class, "FrontL");
        BackL = hardwareMap.get(DcMotor.class, "BackL");
        FrontR = hardwareMap.get(DcMotor.class, "FrontR");
        BackR = hardwareMap.get(DcMotor.class, "BackR");
        ColorSensorL = hardwareMap.get(ColorSensor.class, "ColorSensorL");

        // Put initialization blocks here.
        FrontL.setDirection(DcMotorSimple.Direction.REVERSE);
        BackL.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        FrontL.setPower(1);
        FrontR.setPower(1);
        BackL.setPower(1);
        BackR.setPower(1);
        while (opModeIsActive()) {
            colorValue = Color.rgb(ColorSensorL.red(), ColorSensorL.green(), ColorSensorL.blue());
            telemetry.addData("Hue", JavaUtil.colorToHue(colorValue));
            // Put loop blocks here.
            if (JavaUtil.colorToHue(colorValue) > 330){
                FrontL.setPower(-1);
                FrontR.setPower(-1);
                BackL.setPower(-1);
                BackR.setPower(-1);
                sleep(100);
                FrontL.setPower(0);
                FrontR.setPower(0);
                BackL.setPower(0);
                BackR.setPower(0);
            }
            telemetry.update();
        }
    }
}