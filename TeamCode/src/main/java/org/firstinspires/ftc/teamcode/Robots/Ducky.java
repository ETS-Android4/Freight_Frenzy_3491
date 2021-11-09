// Hardware and Autonomous set-up for Robot for 2021-2022 Freight Frenzy

package org.firstinspires.ftc.teamcode.Robots;

import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Ducky {

    // Declaring Drivebase Motor Variables
    public DcMotor FrontLeft, BackLeft, FrontRight, BackRight;

    // Declaring Mechanisms
    public DcMotor ArmRotator;
    public CRServo Collector;

    public CRServo CarouselSpinner;

    // Declaring opMode Variables
    HardwareMap hwMap;
    Telemetry telemetry;

    // Declaring sensors
    public BNO055IMU imu;
    public static final double BACK_WHEEL_POWER_REDUCTION = 0.7559;

    // Encoder + Wheel Declaration
    public static final double WHEEL_GEAR_RATIO = 2; // 2:1 ratio

    public static final double CART_WHEEL_PULSES_PER_INCH = 34.2/ WHEEL_GEAR_RATIO; // Num of Pulses per inch travelled with Cart Wheel
    public int Cart_Wheel_Distance; // To be used in functions for setTargetPosition

    public static final int ARM_COLLECTING_ENCODER_PULSES = 0;
    public static final int ARM_BOTTOM_LEVEL_ENCODER_PULSES = 0;
    public static final int ARM_MID_LEVEL_ENCODER_PULSES = 0;
    public static final int ARM_TOP_LEVEL_ENCODER_PULSES = 0;

    // Class Constructor
    public Ducky(){

    }

    public void init(HardwareMap ahwMap, Telemetry a_telemetry)  {

        // Calling variable
        hwMap = ahwMap;
        telemetry = a_telemetry;

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

        // Mechanisms - Motors
        ArmRotator = hwMap.dcMotor.get("armRotator");

        // Define Servos
        Collector = hwMap.crservo.get("collector");
        CarouselSpinner = hwMap.crservo.get("carouselSpinner");

        // Initialize Servos
        Collector.setPower(0);
        CarouselSpinner.setPower(0);

        // Motor set Power at init
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);

        // Sensors
        imu = hwMap.get(BNO055IMU.class, "imu");


        // Setting Motors to run with/ without Encoders - (RUN_WITHOUT_ENCODER/ RUN_USING_ENCODER)
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ArmRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Autonomous
     */
    // Robot Driving (Encoder)
    public void DriveForward_Encoder(int Distance, double speed){
        Cart_Wheel_Distance = (int)(Distance*CART_WHEEL_PULSES_PER_INCH);

        BackLeft.setTargetPosition(Cart_Wheel_Distance);
        BackRight.setTargetPosition(Cart_Wheel_Distance);

        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontLeft.setPower(-speed);
        BackLeft.setPower(-speed/ BACK_WHEEL_POWER_REDUCTION);
        FrontRight.setPower(-speed);
        BackRight.setPower(-speed/ BACK_WHEEL_POWER_REDUCTION);

        while (BackLeft.isBusy() || BackRight.isBusy()) {
//            telemetry.addData("Driving Forward, Encoder Pulses Left (Cart)",
//                    Cart_Wheel_Distance - BackLeft.getCurrentPosition());
//            telemetry.update();
        }

        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);

        Stop_Encoder();
    }
    public void DriveBackward_Encoder(int Distance, double speed){
        Cart_Wheel_Distance = (int)(Distance*CART_WHEEL_PULSES_PER_INCH);

        BackLeft.setTargetPosition(Cart_Wheel_Distance);
        BackRight.setTargetPosition(Cart_Wheel_Distance);

        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontLeft.setPower(-speed);
        BackLeft.setPower(-speed/ BACK_WHEEL_POWER_REDUCTION);
        FrontRight.setPower(-speed);
        BackRight.setPower(-speed/ BACK_WHEEL_POWER_REDUCTION);

        while (BackLeft.isBusy() || BackRight.isBusy()) {
            telemetry.addData("Driving Backwards, Encoder Pulses Left (Cart)",
                    0);
            telemetry.update();
        }

        Stop_Encoder();
    }
    public void Stop_Encoder(){
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setPower(0);
        FrontRight.setPower(0);

        telemetry.addData("Robot stopped. Encoder Pulses (Cart)",
                        BackLeft.getCurrentPosition());
        telemetry.update();
    }

    // Robot Driving (Power and Time only)
    public void DriveForward_Power(double speed, int milliseconds){
        FrontLeft.setPower(speed);
        BackLeft.setPower(speed/ BACK_WHEEL_POWER_REDUCTION);
        FrontRight.setPower(speed);
        BackRight.setPower(speed/ BACK_WHEEL_POWER_REDUCTION);
        sleep(milliseconds);
        Stop_Power();
    }
    public void DriveBackward_Power(double speed, int milliseconds) throws InterruptedException{
        FrontLeft.setPower(-speed);
        BackLeft.setPower(-speed/ BACK_WHEEL_POWER_REDUCTION);
        FrontRight.setPower(-speed);
        BackRight.setPower(-speed/ BACK_WHEEL_POWER_REDUCTION);
        Thread.sleep(milliseconds);
        Stop_Power();
    }
    public void TurnLeft_Power(double speed, int milliseconds) throws InterruptedException{
        FrontLeft.setPower(-speed);
        BackLeft.setPower(-speed/ BACK_WHEEL_POWER_REDUCTION);
        FrontRight.setPower(speed);
        BackRight.setPower(speed/ BACK_WHEEL_POWER_REDUCTION);
        Thread.sleep(milliseconds);
        Stop_Power();
    }
    public void TurnRight_Power(double speed, int milliseconds) throws InterruptedException{
        FrontLeft.setPower(speed);
        BackLeft.setPower(speed/ BACK_WHEEL_POWER_REDUCTION);
        FrontRight.setPower(-speed);
        BackRight.setPower(-speed/ BACK_WHEEL_POWER_REDUCTION);
        Thread.sleep(milliseconds);
        Stop_Power();
    }
    public void Stop_Power(){
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);
    }

    /**
     * Mechanism Functions
     */
    // Collector
    public void CollectorOn(){
        Collector.setPower(1);
    }
    public void CollectorReverse(){
        Collector.setPower(-1);
    }
    public void CollectorOff(){
        Collector.setPower(0);
    }

    // Arm Rotator
    public void RotateArm(double power){
        ArmRotator.setPower(power);
    }
//
//    public void ArmCollecting(){
//        ArmRotator.setTargetPosition(ARM_COLLECTING_ENCODER_PULSES);
//        ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        ArmRotator.setPower(0.5);
//
//        while (ArmRotator.isBusy()) {
//            telemetry.addData("Arm Rotating, Position: Collecting" +
//                            " Encoder Pulses Left",
//                    ARM_COLLECTING_ENCODER_PULSES - ArmRotator.getCurrentPosition());
//            telemetry.update();
//        }
//
//        ArmRotator.setPower(0);
//    }
//    public void ArmBottomLevel(){
//        ArmRotator.setTargetPosition(ARM_BOTTOM_LEVEL_ENCODER_PULSES);
//        ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        ArmRotator.setPower(0.5);
//
//        while (ArmRotator.isBusy()) {
//            telemetry.addData("Arm Rotating, Position: Bottom Level" +
//                            " Encoder Pulses Left",
//                    ARM_BOTTOM_LEVEL_ENCODER_PULSES - ArmRotator.getCurrentPosition());
//            telemetry.update();
//        }
//
//        ArmRotator.setPower(0);
//    }
//    public void ArmMidLevel(){
//        ArmRotator.setTargetPosition(ARM_MID_LEVEL_ENCODER_PULSES);
//        ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        ArmRotator.setPower(0.5);
//
//        while (ArmRotator.isBusy()) {
//            telemetry.addData("Arm Rotating, Position: Mid Level" +
//                            " Encoder Pulses Left",
//                    ARM_MID_LEVEL_ENCODER_PULSES - ArmRotator.getCurrentPosition());
//            telemetry.update();
//        }
//
//        ArmRotator.setPower(0);
//    }
//    public void ArmTopLevel(){
//        ArmRotator.setTargetPosition(ARM_TOP_LEVEL_ENCODER_PULSES);
//        ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        ArmRotator.setPower(0.5);
//
//        while (ArmRotator.isBusy()) {
//            telemetry.addData("Arm Rotating, Position: Top Level" +
//                            " Encoder Pulses Left",
//                    ARM_TOP_LEVEL_ENCODER_PULSES - ArmRotator.getCurrentPosition());
//            telemetry.update();
//        }
//
//        ArmRotator.setPower(0);
//    }

    // Carousel Spinner
    public void CarouselSpinnerOn(){
        CarouselSpinner.setPower(1);
    }
    public void CarouselSpinnerReverse(){
        CarouselSpinner.setPower(-1);
    }
    public void CarouselSpinnerOff(){
        CarouselSpinner.setPower(0);
    }

}