package org.firstinspires.ftc.teamcode.Competition.z_SetUp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Robots.Beyonce;
import org.firstinspires.ftc.teamcode.opMode_Support.AutoOpMode;

@Disabled

@Autonomous
public class TestColourSense extends AutoOpMode {
    org.firstinspires.ftc.teamcode.Robots.Beyonce Beyonce;

    private VoltageSensor ExpansionHub2_VoltageSensor;

    @Override
    public void runOp() {
        ExpansionHub2_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Expansion Hub 2");

        Beyonce beyonce = new Beyonce();

        ColorSensor colorSensorL;
        colorSensorL = (ColorSensor) hardwareMap.get("ColourSensorL");

        boolean red = opModeIsActive() && 120 < colorSensorL.red();

        waitForStart();

        colorSensorL.enableLed(true);

        while (opModeIsActive() ){
            telemetry.addData("red", colorSensorL.red());
        }
        beyonce.Stop();
    }

    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}