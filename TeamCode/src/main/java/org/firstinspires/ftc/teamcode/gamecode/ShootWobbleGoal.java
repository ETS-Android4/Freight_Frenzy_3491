package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Beyonce;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;

@Autonomous
public class ShootWobbleGoal extends AutoOpMode {
    @Override
    public void runOp() throws InterruptedException {
        Beyonce beyonce = new Beyonce();

        //beyonce.init();

        telemetry.addData("status", "init");

        beyonce.RingPusherRetract();

        waitForStart();

        beyonce.ShooterOn();
        sleep(5000);

        beyonce.Shoot();
        beyonce.Shoot();
        beyonce.Shoot();

        beyonce.ShooterOff();

//        beyonce.StrafeRight(0.2);
//        sleep(400);
//        beyonce.Stop();

        beyonce.GrabberDown();
    }
}
