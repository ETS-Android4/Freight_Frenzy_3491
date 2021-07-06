package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robots.Beyonce;
import org.firstinspires.ftc.teamcode.newhardware.Motor;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;

@Disabled

@Autonomous
public class TestIMU extends AutoOpMode {
    org.firstinspires.ftc.teamcode.Robots.Beyonce Beyonce;

    public BNO055IMU imu;
    public void runOp() throws InterruptedException {

        //beyonce.GrabberUp();
        BNO055IMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        waitForStart();

        //yaw = imu.getAngularOrientation(angles.firstAngle);

        while (opModeIsActive()) {






        //telemetry.addData("blue", colorSensorL.blue());
      //  telemetry.addData("green", colorSensorL.green());
        //telemetry.addData("light", colorSensorL.alpha());
       // telemetry.addData("4 colour channels", colorSensorL.argb());
            }

//        beyonce.StrafeLeft(0.2);
//        telemetry.addData("staus", "left");
//        sleep(1000);
//        beyonce.Stop();
//
//        sleep(200);
//
//        beyonce.DriveForward(0.1);
//        telemetry.addData("staus", "forward");
//        sleep(700);
//        beyonce.Stop();
//
//        sleep(300);
//
//        clearTimer(1);
//
//        while (opModeIsActive() && 38 > colorSensorL.red() && getSeconds(1) < 4){
//            telemetry.addData("status", getSeconds(1));
//            telemetry.addData("red", colorSensorL.red());
//
//            beyonce.StrafeRight(0.1);
//        }
//        telemetry.addData("staus", "white");
//        telemetry.addData("red", colorSensorL.red());
//
//        beyonce.Stop();

//        sleep(500);
//
//        beyonce.StrafeLeft(0.1);
//        sleep(1250);
//        beyonce.Stop();
//
//        sleep(200);
//nice
//        beyonce.DriveBackward(0.1);
//        sleep(1300);
//        beyonce.Stop();
//
//        sleep(300);
//
//        beyonce.GrabberDown();
//
//        sleep(600);
//
//        beyonce.DriveForward(0.3);
//        sleep(300);
//        beyonce.Stop();






        //red in between like 30 and 36

    }


}