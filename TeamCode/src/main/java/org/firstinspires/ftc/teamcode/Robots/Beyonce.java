package org.firstinspires.ftc.teamcode.Robots;

import org.firstinspires.ftc.teamcode.Robots.Robot;
import org.firstinspires.ftc.teamcode.newhardware.FXTServo;
import org.firstinspires.ftc.teamcode.newhardware.Motor;

public class Beyonce extends Robot {
    public Motor FrontRight;
    public Motor FrontLeft;
    public Motor BackRight;
    public Motor BackLeft;

    public Motor LinearSlide;
    public FXTServo Grabber;

    public Motor Shooter;
    public FXTServo TargetRamp;

    public FXTServo RingPusher;


    //Declaring stuff
    public Beyonce(){
        //Drivebase
        FrontRight = new Motor("frontR");
        FrontLeft = new Motor("frontL");
        BackRight = new Motor("backR");
        BackLeft = new Motor("backL");

        //Wobble Grabber
        LinearSlide = new Motor("Linear Slide");
        Grabber = new FXTServo("Grabber");

        //Shooter
        Shooter = new Motor("Shooter");
        TargetRamp = new FXTServo("TargetRamp");

        //Hopper
        RingPusher = new FXTServo("RingPusher");

        FrontRight.setMinimumSpeed(0.1);
        FrontLeft.setMinimumSpeed(0.1);
        BackRight.setMinimumSpeed(0.1);
        BackLeft.setMinimumSpeed(0.1);
    }


    //Robot Driving
    public void DriveForward(double speed){
        FrontLeft.setPower(-speed);
        FrontRight.setPower(speed);
        BackLeft.setPower(-speed);
        BackRight.setPower(speed);
    }
    public void DriveBackward(double speed){
        FrontLeft.setPower(speed);
        FrontRight.setPower(-speed);
        BackLeft.setPower(speed);
        BackRight.setPower(-speed);
    }
    public void StrafeLeft(double speed){
        FrontLeft.setPower(speed);
        FrontRight.setPower(speed);
        BackLeft.setPower(-speed);
        BackRight.setPower(-speed);
    }
    public void StrafeRight(double speed){
        FrontLeft.setPower(-speed);
        FrontRight.setPower(-speed);
        BackLeft.setPower(speed);
        BackRight.setPower(speed);
    }
    public void TurnLeft(double speed){
        FrontLeft.setPower(-speed);
        FrontRight.setPower(-speed);
        BackLeft.setPower(-speed);
        BackRight.setPower(-speed);

    }
    public void TurnRight(double speed){
        FrontLeft.setPower(speed);
        FrontRight.setPower(speed);
        BackLeft.setPower(speed);
        BackRight.setPower(speed);
    }

    public void Stop(){
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    public void LinearSlidesUp() {
        LinearSlide.setPower(1);
    }

    public void LinearSlidesDown() {
        LinearSlide.setPower(-1);
    }

    public void LinearSidesStop() {
        LinearSlide.setPower(0);
    }

    public void GrabberUp() {
        Grabber.setPosition(1);
    }

    public void GrabberDown() {
        Grabber.setPosition(0);
    }

    public void ShooterOn() {
        Shooter.setPower(0.75);
    }

    public void ShooterOff() {
        Shooter.setPower(0);
    }

    public void RampLevelOne() {
        TargetRamp.setPosition(0);
    }

    public void RampLevelTwo() {
        TargetRamp.setPosition(0.33);
    }

    public void RampLevelThree() {
        TargetRamp.setPosition(0.66);
    }

    public void RingPusherExtend() {
        RingPusher.setPosition(0.2);
    }

    public void RingPusherRetract() {
        RingPusher.setPosition(0.8);
    }

}
