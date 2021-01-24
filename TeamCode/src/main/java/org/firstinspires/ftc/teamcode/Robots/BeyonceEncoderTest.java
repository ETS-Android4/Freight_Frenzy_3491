package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.newhardware.FXTServo;
import org.firstinspires.ftc.teamcode.newhardware.Motor;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


public class BeyonceEncoderTest {
    public Motor FrontRight;
    public Motor FrontLeft;
    public Motor BackRight;
    public Motor BackLeft;
    public Motor Arm;
    public Motor Led;

    public DcMotorEx Shooter;
    public DcMotor ShooterDcMotor = Shooter;

    public FXTServo Claw;

    public FXTServo Ramp;

    public FXTServo RingPusher;


//    ColorSensor colorSensorL;
//    ColorSensor colorSensorR;

    //Declaring stuff
    public BeyonceEncoderTest(){

        //Drivebase
        FrontRight = new Motor("frontR");
        FrontLeft = new Motor("frontL");
        BackRight = new Motor("backR");
        BackLeft = new Motor("backL");
        Led = new Motor ("LED");


        //Wobble Grabber
        Claw = new FXTServo("Claw");
        Arm = new Motor("Arm");

        //Shooter


//        try {
            ShooterDcMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
//        } catch (NullPointerException) {
//            telemetry.addData("Encoder null test: ", "failed");
//            telemetry.update();
//        }

        //Shooter Ramp
        Ramp = new FXTServo("Ramp");

        //Ring pusher
        RingPusher = new FXTServo("RingPusher");

        FrontRight.setMinimumSpeed(0.1);
        FrontLeft.setMinimumSpeed(0.1);
        BackRight.setMinimumSpeed(0.1);
        BackLeft.setMinimumSpeed(0.1);

        Led.setPower(1);
//        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
/*nice*/FrontRight.setPower(-speed);
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

    //Set Position and Power
    public void ClawOpen() {
        Claw.setPosition(0);
    }
    public void ClawClose() {
        Claw.setPosition(1);
    }

//    public void ShooterOn() {
//        Shooter.setVelocity(200);
//    }
//    public void ShooterOff() {
//        Shooter.setVelocity(0);
//    }

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