package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Beyonce;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;

@Autonomous
public class CompShootPark extends AutoOpMode {

    @Override
    public void runOp() throws InterruptedException {
        Beyonce beyonce = new Beyonce();

        //beyonce.init();

        telemetry.addData("status", "init");

        waitForStart();

        beyonce.ShooterOn();
        sleep(2000);
        beyonce.Shoot();
        sleep(1000);
        beyonce.Shoot();
        sleep(1000);
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
        sleep(2000);
        beyonce.Stop();


    }
}
