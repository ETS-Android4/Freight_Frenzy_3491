package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.BeyonceEncoderTest;
import org.firstinspires.ftc.teamcode.Robots.EncoderBot;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;

@Autonomous
public class driveTest extends AutoOpMode {

    @Override
    public void runOp() throws InterruptedException {
        boolean encoderUsed = true;
        EncoderBot beyonce1 = new EncoderBot();
        BeyonceEncoderTest beyonce = new BeyonceEncoderTest(hardwareMap);

        //beyonce.init();

        telemetry.addData("status", "init");

        waitForStart();

//
//
//        beyonce.DriveBackward(0.3);
//        sleep(2000);
//        beyonce.Stop();
//
//        beyonce.DriveForward(0.3);
//        sleep(2000);
//        beyonce.Stop();


    }
}
