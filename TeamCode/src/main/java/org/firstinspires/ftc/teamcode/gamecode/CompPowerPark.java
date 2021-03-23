package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robots.Beyonce;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;

@Autonomous
public class CompPowerPark extends AutoOpMode {

    @Override
    public void runOp() throws InterruptedException {
        Beyonce beyonce = new Beyonce();
        BNO055IMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float Angle = angles.firstAngle;


        //beyonce.init();

        telemetry.addData("status", "init");

        waitForStart();

        beyonce.ShooterOn();
        sleep(2000);
        beyonce.Shoot();
        sleep(1000);

        //turn with IMU here

        beyonce.Shoot();
        sleep(1000);

        //turn with IMU here

        beyonce.Shoot();
        sleep(1000);
        beyonce.ShooterOff();

        beyonce.ClawClose();
        beyonce.StrafeLeft(0.2);
        sleep(500);
        beyonce.Stop();

        sleep(200);

        beyonce.DriveForward(0.5);
        sleep(1000);
        beyonce.Stop();

        sleep(200);

        beyonce.StrafeRight(0.5);
        sleep(2200);
        beyonce.Stop();


    }
}
