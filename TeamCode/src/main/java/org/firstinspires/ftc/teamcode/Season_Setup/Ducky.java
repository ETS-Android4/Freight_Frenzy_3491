// Hardware and Autonomous set-up for Robot for 2021-2022 Freight Frenzy

package org.firstinspires.ftc.teamcode.Season_Setup;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class Ducky {

    // Declaring Drivebase Motor Variables
    public DcMotorEx FrontLeft, BackLeft, FrontRight, BackRight;

    // Declaring Mechanisms
    public DcMotorEx ArmRotator;
    public CRServo Collector;

    public CRServo CarouselSpinner;

    // Declaring opMode Variables
    HardwareMap hwMap;
    Telemetry telemetry;

    // Declaring sensors
    public BNO055IMU imu;

    // Encoder + Wheel Declaration
    public static final double BACK_WHEEL_POWER_REDUCTION = 0.7559;
    public static final double WHEEL_GEAR_RATIO = 2; // 2:1 ratio

    public static final double WHEEL_PULSES_PER_INCH = 34.2/ WHEEL_GEAR_RATIO; // Num of Pulses per inch travelled with Cart Wheel
    public int Encoder_Distance; // To be used in functions for setTargetPosition

    public static final int ARM_COLLECTING_ENCODER_PULSES = 0;
    public static final int ARM_BOTTOM_LEVEL_ENCODER_PULSES = 0;
    public static final int ARM_MID_LEVEL_ENCODER_PULSES = 0;
    public static final int ARM_TOP_LEVEL_ENCODER_PULSES = 0;

    // EasyOpenCV Setup
    public OpenCvCamera webcam;
    public static int analysis = 0;

    // IMU functions
    public Orientation lastAngles = new Orientation();
    public double currentAngle = 0.0;
    float Yaw_Angle;


    // Class Constructor
    public Ducky(){

    }

    public void init(HardwareMap ahwMap, Telemetry a_telemetry)  {

        // Calling variable
        hwMap = ahwMap;
        telemetry = a_telemetry;

        // Define Drive Motors
        FrontLeft = hwMap.get(DcMotorEx.class,"frontL");
        BackLeft = hwMap.get(DcMotorEx.class,"backL");
        FrontRight = hwMap.get(DcMotorEx.class,"frontR");
        BackRight = hwMap.get(DcMotorEx.class,"backR");

        // Setting Motor Direction
        FrontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        BackLeft.setDirection(DcMotorEx.Direction.REVERSE);
        FrontRight.setDirection(DcMotorEx.Direction.FORWARD);
        BackRight.setDirection(DcMotorEx.Direction.FORWARD);

        // Setting Motor zero power Behaviour
        FrontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        // Mechanisms - Motors
        ArmRotator = hwMap.get(DcMotorEx.class,"armRotator");

        // Mechanisms - Setting Motor Direction
        FrontRight.setDirection(DcMotorEx.Direction.FORWARD);

        // Mechanisms - Setting Motor zero power Behaviour
        ArmRotator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


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

//        // EasyOpenCV Setup
//        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webcam.setPipeline(new Freight_Frenzy_Pipeline.Pipeline());
//        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//            }
//        });


        // Setting Motors to run with/ without Encoders - (RUN_WITHOUT_ENCODER/ RUN_USING_ENCODER)
        BackLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ArmRotator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Autonomous
     */
    // Robot Driving (Encoder)
    public void DriveForward_Encoder(int Distance, double speed) throws InterruptedException {
        Encoder_Distance = (int)(Distance* WHEEL_PULSES_PER_INCH);

        BackLeft.setTargetPosition(Encoder_Distance);
        BackRight.setTargetPosition(Encoder_Distance);

        BackLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        DriveForward_Power(speed);

        while (BackRight.isBusy()) {
            telemetry.addData("Driving Forward, Target Position",
                    Encoder_Distance);
            telemetry.addData("Driving Forward, Encoder Pulses Right",
                    BackRight.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addData("Finish",
                Encoder_Distance + BackRight.getCurrentPosition());
        telemetry.update();

        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);

        BackLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Thread.sleep(5000);

        Stop_Encoder();
    }
    public void DriveBackward_Encoder(int Distance, double speed){
        Encoder_Distance = (int)(Distance* WHEEL_PULSES_PER_INCH);

        BackLeft.setTargetPosition(Encoder_Distance);
        BackRight.setTargetPosition(Encoder_Distance);

        BackLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        DriveBackward_Power(speed);

        while (BackLeft.isBusy() || BackRight.isBusy()) {
            telemetry.addData("Driving Backwards, Encoder Pulses Left (Cart)",
                    0);
            telemetry.update();
        }

        Stop_Encoder();
    }
    public void Stop_Encoder(){
        BackLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Robot stopped. Encoder Pulses (Cart)",
                        BackLeft.getCurrentPosition());
        telemetry.update();
    }

    // Robot Driving (Power only)
    public void DriveForward_Power(double speed) {
        FrontLeft.setPower(speed);
        BackLeft.setPower(speed/ BACK_WHEEL_POWER_REDUCTION);
        FrontRight.setPower(speed);
        BackRight.setPower(speed/ BACK_WHEEL_POWER_REDUCTION);
    }
    public void DriveBackward_Power(double speed) {
        FrontLeft.setPower(-speed);
        BackLeft.setPower(-speed/ BACK_WHEEL_POWER_REDUCTION);
        FrontRight.setPower(-speed);
        BackRight.setPower(-speed/ BACK_WHEEL_POWER_REDUCTION);
    }
    public void Stop_Power() {
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

//    public void ArmCollecting(){
//        ArmRotator.setTargetPosition(ARM_COLLECTING_ENCODER_PULSES);
//        ArmRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
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
//        ArmRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
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
//        ArmRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
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
//        ArmRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
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


    /* IMU Functions */
    public void  resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentAngle = 0;
    }
    public void getAngle() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void DriveForward_Encoder_IMU (int Distance, double speed) {
        Encoder_Distance = (int)(Distance* WHEEL_PULSES_PER_INCH);

        BackLeft.setTargetPosition(Encoder_Distance);
        BackRight.setTargetPosition(Encoder_Distance);

        BackLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        DriveForward_Power(speed);

        while (BackLeft.isBusy() || BackRight.isBusy()) {

            Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;


            // If robot is drifting right - Need to turn left
            if (Yaw_Angle < -5) {
                FrontLeft.setPower(speed-0.05);
                BackLeft.setPower(speed-0.05);
                FrontRight.setPower(speed+0.05);
                BackRight.setPower(speed+0.05);

            // If robot is drifting left - Need to turn right
            } else if (Yaw_Angle > 5) {
                // Turn right.
                FrontLeft.setPower(speed+0.05);
                BackLeft.setPower(speed+0.05);
                FrontRight.setPower(speed-0.05);
                BackRight.setPower(speed-0.05);

            // Countinue Straight
            } else {
                FrontLeft.setPower(speed);
                BackLeft.setPower(speed);
                FrontRight.setPower(speed);
                BackRight.setPower(speed);
            }
        }

        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);

        Stop_Encoder();
    }

    public void TurnRight_Encoder_IMU (double speed) {
        while (Yaw_Angle <= -90) {
            // Update Yaw-Angle variable with current yaw.
            Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            FrontLeft.setPower(speed);
            BackLeft.setPower(speed);
            FrontRight.setPower(-speed);
            BackRight.setPower(-speed);

            // Report yaw orientation to Driver Station.
            telemetry.addData("Yaw value", Yaw_Angle);
            telemetry.update();
        }

        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);

        Stop_Encoder();
    }
}