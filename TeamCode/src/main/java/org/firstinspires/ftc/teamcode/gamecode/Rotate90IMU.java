package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

@Disabled

@Autonomous
public class Rotate90IMU extends AutoOpMode {

    BNO055IMU imu;

    @Override
    public void runOp() throws InterruptedException {

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //IMU paramater setup
        BNO055IMU.Parameters imuParameters;
        Orientation angles;
        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        imuParameters.loggingEnabled = false;


        //IMU setup
        BNO055IMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        float firstAngle;

        waitForStart();

        addTimer();
        while (getSeconds() < 10 && opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            firstAngle = angles.firstAngle;
            telemetry.addData("rotational angle", angles.firstAngle);
            telemetry.update();
        }
    }
}
