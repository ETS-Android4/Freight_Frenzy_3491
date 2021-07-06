package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robots.Beyonce;
import org.firstinspires.ftc.teamcode.Robots.BeyonceEncoderQualcommDcMotorExTest;
import org.firstinspires.ftc.teamcode.Robots.BeyonceEncoderTest;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;

@Disabled

@Autonomous
public class EncoderMotorWhyIsItNotStrafingProperlySolutionToProblemUsingEncoderTelemetry extends AutoOpMode {

    @Override
    public void runOp() throws InterruptedException {
        BeyonceEncoderQualcommDcMotorExTest beyonce = new BeyonceEncoderQualcommDcMotorExTest();

        //beyonce.init();

        telemetry.addData("status", "init");

        waitForStart();

        beyonce.StrafeRight(0.3);
        sleep(400);
        beyonce.Stop();
    }
}






























//nice