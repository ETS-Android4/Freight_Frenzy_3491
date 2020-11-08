package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robots.Beyonce;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;


@Autonomous
public class Test extends AutoOpMode {
    org.firstinspires.ftc.teamcode.Robots.Beyonce Beyonce;

    @Override
    public void runOp() throws InterruptedException {
        Beyonce beyonce = new Beyonce();
        waitForStart();
        beyonce.GrabberUp();
        sleep(1000);
        beyonce.GrabberDown();


    }


}