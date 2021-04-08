package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.newhardware.FXTServo;
import org.firstinspires.ftc.teamcode.newhardware.Motor;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


public class BeyonceEncoderQualcommDcMotorExTest {
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor backRight;
    public DcMotor backLeft;
    public Motor Arm;
    public Motor Led;

    public FXTServo Claw;

    public DcMotor shooter;

    public FXTServo Ramp;

    public FXTServo RingPusher;

    public Motor feeder;
    public Motor ziptiepuller;

    public FXTServo WallHolder;

    ColorSensor colorSensorL;
    ColorSensor colorSensorR;


    //Declaring stuff
    public BeyonceEncoderQualcommDcMotorExTest(){
//        //Drivebase
//        frontRight = hardwareMap.get(DcMotor.class, "frontR");
//        sleep(10000);
//        frontLeft = hardwareMap.get(DcMotor.class, "frontL");
//        backRight = hardwareMap.get(DcMotor.class, "backR");
//        backLeft = hardwareMap.get(DcMotor.class, "backL");
//        Led = new Motor ("LED");



        //Wobble Grabber
        Claw = new FXTServo("Claw");
        Arm = new Motor("Arm");


        //Shooter
        //shooter = hardwareMap.get(DcMotor.class, "Shooter");

        //Shooter Ramp
        Ramp = new FXTServo("Ramp");

        //Ring pusher
        RingPusher = new FXTServo("RingPusher");

        ziptiepuller= new Motor("ziptiepuller");
        feeder = new Motor("feeder");


        //Wobble Grabber
        Claw = new FXTServo("Claw");
        Arm = new Motor("Arm");

        //Shooter Ramp
        Ramp = new FXTServo("Ramp");

        WallHolder = new FXTServo("WallHolder");

        //Ring pusher
        RingPusher = new FXTServo("RingPusher");



//        FrontRight.setMinimumSpeed(0.1);
//        FrontLeft.setMinimumSpeed(0.1);
//        BackRight.setMinimumSpeed(0.1);
//        BackLeft.setMinimumSpeed(0.1);
        sleep(10000);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ((DcMotorEx) shooter).setVelocity(0);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ((DcMotorEx) frontLeft).setVelocity(0);
        ((DcMotorEx) frontRight).setVelocity(0);
        ((DcMotorEx) backLeft).setVelocity(0);
        ((DcMotorEx) backRight).setVelocity(0);


        Led.setPower(1);
    }

    //Robot Driving
    public void DriveForward(double speed){
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);
        telemetry.addData("frontLeft:", "\nfrontRight", "\nbackLeft", "\nbackRight", ((DcMotorEx) frontLeft).getVelocity(), ((DcMotorEx)frontRight).getVelocity(), ((DcMotorEx) backLeft).getVelocity(), ((DcMotorEx)backRight).getVelocity());
        telemetry.update();
    }
    public void DriveBackward(double speed){
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);
        telemetry.addData("frontLeft:", "\nfrontRight", "\nbackLeft", "\nbackRight", ((DcMotorEx) frontLeft).getVelocity(), ((DcMotorEx)frontRight).getVelocity(), ((DcMotorEx) backLeft).getVelocity(), ((DcMotorEx)backRight).getVelocity());
        telemetry.update();
    }
    public void StrafeLeft(double speed){
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(-speed);
        backRight.setPower(-speed);
        telemetry.addData("frontLeft:", "\nfrontRight", "\nbackLeft", "\nbackRight", ((DcMotorEx) frontLeft).getVelocity(), ((DcMotorEx)frontRight).getVelocity(), ((DcMotorEx) backLeft).getVelocity(), ((DcMotorEx)backRight).getVelocity());
        telemetry.update();
    }
    public void StrafeRight(double speed){
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);
        telemetry.addData("frontLeft:", "\nfrontRight", "\nbackLeft", "\nbackRight", ((DcMotorEx) frontLeft).getVelocity(), ((DcMotorEx)frontRight).getVelocity(), ((DcMotorEx) backLeft).getVelocity(), ((DcMotorEx)backRight).getVelocity());
        telemetry.update();
    }
    public void TurnLeft(double speed){
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(-speed);
        telemetry.addData("frontLeft:", "\nfrontRight", "\nbackLeft", "\nbackRight", ((DcMotorEx) frontLeft).getVelocity(), ((DcMotorEx)frontRight).getVelocity(), ((DcMotorEx) backLeft).getVelocity(), ((DcMotorEx)backRight).getVelocity());
        telemetry.update();
    }
    public void TurnRight(double speed){
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);
        telemetry.addData("frontLeft:", "\nfrontRight", "\nbackLeft", "\nbackRight", ((DcMotorEx) frontLeft).getVelocity(), ((DcMotorEx)frontRight).getVelocity(), ((DcMotorEx) backLeft).getVelocity(), ((DcMotorEx)backRight).getVelocity());
        telemetry.update();
    }
    public void Stop(){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        telemetry.addData("frontLeft:", "\nfrontRight", "\nbackLeft", "\nbackRight", ((DcMotorEx) frontLeft).getVelocity(), ((DcMotorEx)frontRight).getVelocity(), ((DcMotorEx) backLeft).getVelocity(), ((DcMotorEx)backRight).getVelocity());
        telemetry.update();
    }

    //Set Position and Power
    public void ClawOpen() {
        Claw.setPosition(0);
    }
    public void ClawClose() {
        Claw.setPosition(1);
    }

    public void ShooterOn() {
        ((DcMotorEx) shooter).setVelocity(500);
    }
    public void ShooterOff() {
        ((DcMotorEx) shooter).setVelocity(0);
    }
    public double getShooterVelocity() {
        return ((DcMotorEx) shooter).getVelocity();
    }


    private double position = 0;
    public void moveRamp(double power) {
        position = position + (power / 10);
        if (position < 0.2) {position = 0.2;}
        if (position > 0.8) {position = 0.8;}
        Ramp.setPosition(power);
    }

    public void ArmDown(double power) {
        Arm.setPower(power);
    }

    //Ring pusher method
    public void Shoot() {
        RingPusherExtend();
        sleep(1250);
        RingPusherRetract();
        sleep(750);
        sleep(2500);
    }
    public void RingPusherExtend() {
        RingPusher.setPosition(0.2);
    }
    public void RingPusherRetract() {
        RingPusher.setPosition(0.8);
    }
}