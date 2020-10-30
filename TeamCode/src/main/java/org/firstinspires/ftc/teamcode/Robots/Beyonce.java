package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.newhardware.FXTServo;
import org.firstinspires.ftc.teamcode.newhardware.Motor;

public class Beyonce extends Robot {
    private Motor frontLeft;
    private Motor frontRight;
    private Motor backLeft;
    private Motor backRight;
    private Motor linearSlide;
    private Motor shooter;

    //servos
    private FXTServo grabber;
    private FXTServo targetRamp;
    private FXTServo ringPusher;

    public Beyonce(){
        frontRight = new Motor("frontR");
        frontRight = RC.h.get(Motor.class, "frontR");
        backLeft = RC.h.get(Motor.class, "backL");
        backRight = RC.h.get(Motor.class, "backR");
        linearSlide = RC.h.get(Motor.class, "Linear Slide"); //done
        shooter = RC.h.get(Motor.class, "Shooter");

        grabber = RC.h.get(FXTServo.class, "Grabber"); //done
        targetRamp = RC.h.get(FXTServo.class, "TargetRamp"); //done
        ringPusher = RC.h.get(FXTServo.class, "RingPusher"); //done

    }

    public void init(){
        //put things u want to do on init here
    }


    public void LinearSlidesUp(){ linearSlide.setPower(1);}


}
