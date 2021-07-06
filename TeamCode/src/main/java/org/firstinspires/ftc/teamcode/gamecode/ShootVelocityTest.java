package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robots.Beyonce;
import org.firstinspires.ftc.teamcode.Robots.BeyonceEncoderTest;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;

@Disabled

@Autonomous
public class ShootVelocityTest extends AutoOpMode {

    @Override
    public void runOp() throws InterruptedException {
        BeyonceEncoderTest beyonce = new BeyonceEncoderTest();

        //beyonce.init();

        telemetry.addData("status", "init");

        waitForStart();

//        beyonce.ShooterOn();
        sleep(5000);

//        beyonce.ShooterOff();

    }
}
