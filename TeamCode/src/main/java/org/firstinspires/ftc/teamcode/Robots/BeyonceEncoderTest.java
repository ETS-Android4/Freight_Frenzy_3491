package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.newhardware.FXTServo;
import org.firstinspires.ftc.teamcode.newhardware.Motor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static android.os.SystemClock.sleep;


public class BeyonceEncoderTest {

    static final double COUNTS_PER_MOTOR_REV = 1440 ;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0 ;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0 ;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public DcMotorEx FrontRight;
    public DcMotorEx FrontLeft;
    public DcMotorEx BackRight;
    public DcMotorEx BackLeft;


    public Motor LinearSlide;
    public FXTServo Grabber;

    public Motor Shooter;
    //public FXTServo TargetRamp;

    public FXTServo RingPusher;

    ColorSensor colorSensorL;
    ColorSensor colorSensorR;



    HardwareMap hwMap = null;
    public BeyonceEncoderTest(HardwareMap ahwMap){
        hwMap = ahwMap;

        //Drivebase
        FrontRight = hwMap.get(DcMotorEx.class, "frontR");
        FrontLeft = hwMap.get(DcMotorEx.class, "frontL");
        BackRight = hwMap.get(DcMotorEx.class, "backR");
        BackLeft = hwMap.get(DcMotorEx.class, "backR");

        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

}
