 package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Robots.Robot;
import org.firstinspires.ftc.teamcode.newhardware.FXTCRServo;
import org.firstinspires.ftc.teamcode.newhardware.FXTServo;
import org.firstinspires.ftc.teamcode.newhardware.Motor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import static android.os.SystemClock.sleep;


public class Beyonce {
    public Motor FrontRight;
    public Motor FrontLeft;
    public Motor BackRight;
    public Motor BackLeft;
    public Motor feeder;
    public Motor ziptiepuller;
    public Motor Arm;
    public FXTCRServo BeatInStick;
    //public Motor Led;

    public FXTServo Claw;

    public Motor Shooter;

    public FXTServo Ramp;

    public FXTServo WallHolder;


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
       // Led = new Motor ("LED");

        ziptiepuller = new Motor("ziptiepuller");
        feeder = new Motor("feeder");

        BeatInStick = new FXTCRServo("BeatinStick");

        //Wobble Grabber
        Claw = new FXTServo("Claw");
        Arm = new Motor("Arm");

        //Shooter
        Shooter = new Motor("Shooter");

        //Shooter Ramp
        Ramp = new FXTServo("Ramp");

        WallHolder = new FXTServo("WallHolder");

        //Ring pusher
        RingPusher = new FXTServo("RingPusher");

        FrontRight.setMinimumSpeed(0.1);
        FrontLeft.setMinimumSpeed(0.1);
        BackRight.setMinimumSpeed(0.1);
        BackLeft.setMinimumSpeed(0.1);

        //Led.setPower(1);
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

    public void Beat(double power){
        BeatInStick.setPower(power);
    }

    //Set Position and Power
    public void ClawOpen() {
        Claw.setPosition(0);
    }
    public void ClawClose() {
        Claw.setPosition(1);
    }

    public void ShooterOn() {
        Shooter.setPower(1);
    }
    public void ShooterOff() {
        Shooter.setPower(0);
    }

    private double position = 0;
    public void moveRamp(double power) {
        if (power > 0 && position < 0.8) {
            position = position + 0.01;
        } else if (power < 0 && position > 0.2) {
            position = position - 0.01;
        }
        Ramp.setPosition(position);
    }

    public void ArmDown(double power) {
        Arm.setPower(power);
    }

    //Ring pusher method
    public void Shoot() {
        RingPusherExtend();
        sleep(1300);
        RingPusherRetract();
        sleep(1300);
    }
    public void RingPusherExtend() {
        RingPusher.setPosition(1);
    }
    public void RingPusherRetract() {
        RingPusher.setPosition(0.45);
    }

    public void feedingOn(){
        feeder.setPower(1);
        ziptiepuller.setPower(1);
    }

    public void feedingOff(){
        feeder.setPower(0);
        ziptiepuller.setPower(0);
    }

    public void HoldWall(){
        WallHolder.setPosition(0.2);
    }
    public void ReleaseWall(){
        WallHolder.setPosition(0.8);
    }
}