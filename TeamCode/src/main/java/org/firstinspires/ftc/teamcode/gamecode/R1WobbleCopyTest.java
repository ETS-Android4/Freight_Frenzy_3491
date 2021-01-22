package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Beyonce;
import org.firstinspires.ftc.teamcode.Robots.BeyonceEncoderTest;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;

@Autonomous
public class R1WobbleCopyTest extends AutoOpMode {

    @Override
    public void runOp() throws InterruptedException {
        BeyonceEncoderTest beyonce = new BeyonceEncoderTest();

        //beyonce.init();

        telemetry.addData("status", "init");

        waitForStart();

        beyonce.StrafeLeft(0.2);
        sleep(400);
        beyonce.Stop();

        sleep(500);

        beyonce.DriveBackward(0.3);
        sleep(1400);
        beyonce.Stop();

        beyonce.DriveForward(0.3);
        sleep(100);
        beyonce.Stop();


    }
}






























//nice