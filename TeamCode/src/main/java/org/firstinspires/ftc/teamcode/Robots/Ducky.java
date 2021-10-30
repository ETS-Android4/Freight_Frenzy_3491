// Hardware and Autonomous set-up for Robot for 2021-2022 Freight Frenzy

package org.firstinspires.ftc.teamcode.Robots;

import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Ducky {

    // Declaring Drivebase Motor Variables
    public DcMotor FrontLeft, BackLeft, FrontRight, BackRight;

    // Declaring Mechanisms
    public DcMotor ArmRotator;
    public Servo Collector;

    public Servo CarouselSpinner;

    // Declaring opMode Variables
    HardwareMap hwMap;

    // Encoder + Wheel Declaration
    public static final double OMNI_WHEEL_PULSES_PER_INCH = 45.3; // Num of Pulses per inch travelled with Omni Wheel
    public static final double CART_WHEEL_PULSES_PER_INCH = 34.2; // Num of Pulses per inch travelled with Cart Wheel
    public int Omni_Wheel_Distance; // To be used in functions for setTargetPosition
    public int Cart_Wheel_Distance; // To be used in functions for setTargetPosition

    // Class Constructor
    public Ducky(){

    }

    public void init(HardwareMap ahwMap)  {

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define Drive Motors
        FrontLeft = hwMap.dcMotor.get("frontL");
        BackLeft = hwMap.dcMotor.get("backL");
        FrontRight = hwMap.dcMotor.get("frontR");
        BackRight = hwMap.dcMotor.get("backR");

        // Setting Motor Direction
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);

//        // Mechanisms - Motors
//        ArmRotator = hwMap.dcMotor.get("armRotator");
//
//        // Define Servos
//        Collector  = hwMap.get(Servo.class, "collector");
//        CarouselSpinner = hwMap.get(Servo.class, "carouselSpinner");
//
//        // Initialize Servos
//        Collector.setPosition(0.5);
//        CarouselSpinner.setPosition(0.5);

        // Motor set Power at init
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);

        // Setting Motors to run with/ without Encoders - (RUN_WITHOUT_ENCODER/ RUN_USING_ENCODER)
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Autonomous
     */
    // Robot Driving (Encoder)
    public void DriveForward_Encoder(int Distance, double speed){
        Omni_Wheel_Distance = (int)(Distance*OMNI_WHEEL_PULSES_PER_INCH);
        Cart_Wheel_Distance = (int)(Distance*CART_WHEEL_PULSES_PER_INCH);

        FrontLeft.setTargetPosition(Omni_Wheel_Distance);
        BackLeft.setTargetPosition(Cart_Wheel_Distance);
        FrontRight.setTargetPosition(Omni_Wheel_Distance);
        BackRight.setTargetPosition(Cart_Wheel_Distance);

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontLeft.setPower(speed);
        BackLeft.setPower(speed/ 0.7559);
        FrontRight.setPower(speed);
        BackRight.setPower(speed/ 0.7559);

        while (FrontLeft.isBusy() || BackLeft.isBusy() || FrontLeft.isBusy() || BackRight.isBusy()) {
            telemetry.addData("Driving Forward, Encoder Pulses Left (Omni)",
                    Omni_Wheel_Distance - FrontLeft.getCurrentPosition());
        }

        Stop_Encoder();
    }
    public void DriveBackward_Encoder(int Distance, double speed){
        Omni_Wheel_Distance = (int)(Distance*OMNI_WHEEL_PULSES_PER_INCH);
        Cart_Wheel_Distance = (int)(Distance*CART_WHEEL_PULSES_PER_INCH);

        FrontLeft.setTargetPosition(Omni_Wheel_Distance);
        BackLeft.setTargetPosition(Cart_Wheel_Distance);
        FrontRight.setTargetPosition(Omni_Wheel_Distance);
        BackRight.setTargetPosition(Cart_Wheel_Distance);

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontLeft.setPower(-speed);
        BackLeft.setPower(-speed/ 0.7559);
        FrontRight.setPower(-speed);
        BackRight.setPower(-speed/ 0.7559);

        while (FrontLeft.isBusy() || BackLeft.isBusy() || FrontLeft.isBusy() || BackRight.isBusy()) {
            telemetry.addData("Driving Forward, Encoder Pulses Left (Omni)",
                    Omni_Wheel_Distance - FrontLeft.getCurrentPosition());
        }

        Stop_Encoder();
    }
    public void TurnLeft_Encoder(int Angle, double speed){
        Omni_Wheel_Distance = (int)(Angle*OMNI_WHEEL_PULSES_PER_INCH);
        Cart_Wheel_Distance = (int)(Angle*CART_WHEEL_PULSES_PER_INCH);

        FrontLeft.setTargetPosition(-Omni_Wheel_Distance);
        BackLeft.setTargetPosition(-Cart_Wheel_Distance);
        FrontRight.setTargetPosition(Omni_Wheel_Distance);
        BackRight.setTargetPosition(Cart_Wheel_Distance);

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontLeft.setPower(-speed);
        BackLeft.setPower(-speed/ 0.7559);
        FrontRight.setPower(speed);
        BackRight.setPower(speed/ 0.7559);

        while (FrontLeft.isBusy() || BackLeft.isBusy() || FrontLeft.isBusy() || BackRight.isBusy()) {
            telemetry.addData("Driving Forward, Encoder Pulses Left (Omni)",
                    Omni_Wheel_Distance - FrontLeft.getCurrentPosition());
        }

        Stop_Encoder();
    }
    public void TurnRight_Encoder(int Angle, double speed){
        Omni_Wheel_Distance = (int)(Angle*OMNI_WHEEL_PULSES_PER_INCH);
        Cart_Wheel_Distance = (int)(Angle*CART_WHEEL_PULSES_PER_INCH);

        FrontLeft.setTargetPosition(Omni_Wheel_Distance);
        BackLeft.setTargetPosition(Cart_Wheel_Distance);
        FrontRight.setTargetPosition(-Omni_Wheel_Distance);
        BackRight.setTargetPosition(-Cart_Wheel_Distance);

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontLeft.setPower(speed);
        BackLeft.setPower(speed/ 0.7559);
        FrontRight.setPower(-speed);
        BackRight.setPower(-speed/ 0.7559);

        while (FrontLeft.isBusy() || BackLeft.isBusy() || FrontLeft.isBusy() || BackRight.isBusy()) {
            telemetry.addData("Driving Forward, Encoder Pulses Left (Omni)",
                    Omni_Wheel_Distance - FrontLeft.getCurrentPosition());
        }

        Stop_Encoder();
    }
    public void Stop_Encoder(){
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Robot Driving (Power and Time only)
    public void DriveForward_Power(double speed, int milliseconds){
        FrontLeft.setPower(speed);
        BackLeft.setPower(speed/ 0.7559);
        FrontRight.setPower(speed);
        BackRight.setPower(speed/ 0.7559);
        sleep(milliseconds);
        Stop_Power();
    }
    public void DriveBackward_Power(double speed, int milliseconds){
        FrontLeft.setPower(-speed);
        BackLeft.setPower(-speed/ 0.7559);
        FrontRight.setPower(-speed);
        BackRight.setPower(-speed/ 0.7559);
        sleep(milliseconds);
        Stop_Power();
    }
    public void TurnLeft_Power(double speed, int milliseconds){
        FrontLeft.setPower(-speed);
        BackLeft.setPower(-speed/ 0.7559);
        FrontRight.setPower(speed);
        BackRight.setPower(speed/ 0.7559);
        sleep(milliseconds);
        Stop_Power();
    }
    public void TurnRight_Power(double speed, int milliseconds){
        FrontLeft.setPower(speed);
        BackLeft.setPower(speed/ 0.7559);
        FrontRight.setPower(-speed);
        BackRight.setPower(-speed/ 0.7559);
        sleep(milliseconds);
        Stop_Power();
    }
    public void Stop_Power(){
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);
    }
}