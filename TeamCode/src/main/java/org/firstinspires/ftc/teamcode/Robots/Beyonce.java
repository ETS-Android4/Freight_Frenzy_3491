package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Robots.Robot;
import org.firstinspires.ftc.teamcode.newhardware.FXTCRServo;
import org.firstinspires.ftc.teamcode.newhardware.FXTServo;
import org.firstinspires.ftc.teamcode.newhardware.Motor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import static android.os.SystemClock.sleep;


public class Beyonce {
    public Motor FrontRight;
    public Motor FrontLeft;
    public Motor BackRight;
    public Motor BackLeft;
    public Motor Arm;

    public FXTServo Claw;

    public Motor Shooter;

    public FXTCRServo TargetRamp;

    public FXTServo RingPusher;

    ColorSensor colorSensorL;
    ColorSensor colorSensorR;

    //Declaring stuff
    public Beyonce(){
        //Drivebase
        FrontRight = new Motor("frontR");
        FrontLeft = new Motor("frontL");
        BackRight = new Motor("backR");
        BackLeft = new Motor("backL");

        //Wobble Grabber
        Claw = new FXTServo("Claw");
        Arm = new Motor("Arm");


        //Shooter
        Shooter = new Motor("Shooter");
        TargetRamp = new FXTCRServo("Ramp");

        //Hopper
        RingPusher = new FXTServo("RingPusher");

        FrontRight.setMinimumSpeed(0.1);
        FrontLeft.setMinimumSpeed(0.1);
        BackRight.setMinimumSpeed(0.1);
        BackLeft.setMinimumSpeed(0.1);
    }


    public void init(){
        ClawOpen();
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

    public void ArmsUp() {
        Arm.setPower(1);
    }

    public void ArmsDown() {
        Arm.setPower(-1);
    }

    public void ArmsStop() {
        Arm.setPower(0);
    }

    public void ClawOpen() {
        Claw.setPosition(1);
    }

    public void Clawclose() {
        Claw.setPosition(0);
    }

    public void ShooterOn() {
        Shooter.setPower(1);
    }

    public void ShooterOff() {
        Shooter.setPower(0);
    }

    public void Shoot() {
        RingPusherExtend();
        sleep(750);
        RingPusherRetract();
        sleep(750);
        sleep(2500);
    }


//
//    public void RampLevelOne() {TargetRamp.setPosition(0); }
//
//    public void RampLevelTwo() {
//        TargetRamp.setPosition(0.33);
//    }
//
//    public void RampLevelThree() {
//        TargetRamp.setPosition(0.66);
//    }
    //sets ramp level
//    public void setRampLevel(int level) {
//        if (level == 1) {
//            RampLevelOne();
//        }
//        else if (level == 2) {
//            RampLevelTwo();
//        }
//        else if (level == 3) {
//            RampLevelThree();
//        }
//    }

    public void RingPusherExtend() {
        RingPusher.setPosition(0.2);
    }

    public void RingPusherRetract() {
        RingPusher.setPosition(0.8);
    }

}
