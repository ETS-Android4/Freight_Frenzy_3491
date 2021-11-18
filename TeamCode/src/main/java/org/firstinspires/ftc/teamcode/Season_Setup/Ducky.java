// Hardware and Autonomous set-up for Robot for 2021-2022 Freight Frenzy

package org.firstinspires.ftc.teamcode.Season_Setup;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


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

    // Motor Power Variables
    public double leftPower;
    public double rightPower;

    // Encoder + Wheel Declaration
    public static final double BACK_WHEEL_POWER_REDUCTION = 0.7559;
    public static final double WHEEL_GEAR_RATIO = 2; // 2:1 ratio

    public static final double WHEEL_PULSES_PER_INCH = 34.2/ WHEEL_GEAR_RATIO; // Num of Pulses per inch travelled with Cart Wheel
    public int Encoder_Distance; // To be used in functions for setTargetPosition

    public static final int ARM_COLLECTING_ENCODER_PULSES = 0;
    public static final int ARM_BOTTOM_LEVEL_ENCODER_PULSES = 362;
    public static final int ARM_MID_LEVEL_ENCODER_PULSES = 339;
    public static final int ARM_TOP_LEVEL_ENCODER_PULSES = 297;

    // EasyOpenCV Setup
    public OpenCvCamera webcam;
    public static int analysis = 0;

    // IMU functions
    public float Yaw_Angle;
    public Orientation angles;
    public Acceleration gravity;
//    public static final double TURN_ANGLE_TOLERANCE = 0.2;

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

        // Setting Motors to run with/ without Encoders - (RUN_WITHOUT_ENCODER/ RUN_USING_ENCODER)
        BackLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ArmRotator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        // IMU
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);


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
    }



    //----------------------------------------------------------------------------------------------
    // Driving Functions
    //----------------------------------------------------------------------------------------------

    // IMU and Encoders (where applicable)
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
                if (rightPower < speed+0.2){
                    leftPower -= 0.05;
                    rightPower += 0.05;
                }

                FrontLeft.setPower(leftPower);
                BackLeft.setPower(leftPower);
                FrontRight.setPower(rightPower);
                BackRight.setPower(rightPower);

            // If robot is drifting left - Need to turn right
            } else if (Yaw_Angle > 5) {
                if (leftPower < speed+0.2){
                    leftPower += 0.05;
                    rightPower -= 0.05;
                }

                FrontLeft.setPower(leftPower);
                BackLeft.setPower(leftPower);
                FrontRight.setPower(rightPower);
                BackRight.setPower(rightPower);

            // Continue Straight
            } else {
                if (leftPower >= speed+0.05) {
                    leftPower -= 0.05;
                }
                if (leftPower <= speed-0.05) {
                    leftPower += 0.05;
                }
                if (rightPower >= speed+0.05) {
                    rightPower -= 0.05;
                }
                if (rightPower <= speed-0.05) {
                    rightPower += 0.05;
                }

                FrontLeft.setPower(leftPower);
                BackLeft.setPower(leftPower);
                FrontRight.setPower(rightPower);
                BackRight.setPower(rightPower);
            }

            // Telemetry Update
            telemetry.addData("Driving Backward, Target Position",
                    Encoder_Distance);
            telemetry.addData("Driving Backward, Encoder Pulses Left",
                    BackLeft.getCurrentPosition());
            telemetry.addData("Driving Backward, Encoder Pulses Right",
                    BackRight.getCurrentPosition());
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Yaw value", Yaw_Angle);

            telemetry.update();
        }

        Stop_Encoder();
    }
    public void DriveBackward_Encoder_IMU (int Distance, double speed) {
        Encoder_Distance = (int)(Distance* WHEEL_PULSES_PER_INCH);

        BackLeft.setTargetPosition(Encoder_Distance);
        BackRight.setTargetPosition(Encoder_Distance);

        BackLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        DriveBackward_Power(speed);

        while (BackLeft.isBusy() || BackRight.isBusy()) {

            Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            // If robot is drifting right - Need to turn left
            if (Yaw_Angle < -5) {
                if (leftPower < speed+0.2){
                    leftPower += 0.05;
                    rightPower -= 0.05;
                }

                FrontLeft.setPower(leftPower);
                BackLeft.setPower(leftPower);
                FrontRight.setPower(rightPower);
                BackRight.setPower(rightPower);

            // If robot is drifting left - Need to turn right
            } else if (Yaw_Angle > 5) {
                if (rightPower < speed+0.2){
                    leftPower -= 0.05;
                    rightPower += 0.05;
                }

                FrontLeft.setPower(leftPower);
                BackLeft.setPower(leftPower);
                FrontRight.setPower(rightPower);
                BackRight.setPower(rightPower);

            // Continue Straight
            } else {
                if (leftPower >= speed+0.05) {
                    leftPower -= 0.05;
                }
                if (leftPower <= speed-0.05) {
                    leftPower += 0.05;
                }
                if (rightPower >= speed+0.05) {
                    rightPower -= 0.05;
                }
                if (rightPower <= speed-0.05) {
                    rightPower += 0.05;
                }

                FrontLeft.setPower(leftPower);
                BackLeft.setPower(leftPower);
                FrontRight.setPower(rightPower);
                BackRight.setPower(rightPower);
            }

            // Telemetry Update
            telemetry.addData("Driving Backward, Target Position",
                    Encoder_Distance);
            telemetry.addData("Driving Backward, Encoder Pulses Left",
                    BackLeft.getCurrentPosition());
            telemetry.addData("Driving Backward, Encoder Pulses Right",
                    BackRight.getCurrentPosition());
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Yaw value", Yaw_Angle);

            telemetry.update();
        }

        Stop_Encoder();
    }
    public void TurnLeft_IMU (double angle, double speed) throws InterruptedException {
        angle = Yaw_Angle+angle;

        while (Yaw_Angle <= angle) {
            // Update Yaw-Angle variable with current yaw.
            Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            TurnLeft_Power(speed);

            // Telemetry Update
            telemetry.addData("Target Yaw value", angle);
            telemetry.addData("Current Yaw value", Yaw_Angle);
            telemetry.update();
        }
        Thread.sleep(200);

        while (Yaw_Angle > angle) {
            // Update Yaw-Angle variable with current yaw.
            Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            TurnRight_Power(0.1);

            // Report yaw orientation to Driver Station.
            telemetry.addData("Robot overturned, Target Yaw value", angle);
            telemetry.addData("Current Yaw value", Yaw_Angle);
            telemetry.update();
        }

        Stop_Encoder();
    }
    public void TurnRight_IMU (double angle, double speed) throws InterruptedException {
        angle = Yaw_Angle+ -angle;

        while (Yaw_Angle >= angle) {
            // Update Yaw-Angle variable with current yaw.
            Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            TurnRight_Power(speed);

            // Telemetry Update
            telemetry.addData("Target Yaw value", angle);
            telemetry.addData("Current Yaw value", Yaw_Angle);
            telemetry.update();
        }
        Thread.sleep(200);

        while (Yaw_Angle < angle) {
            // Update Yaw-Angle variable with current yaw.
            Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            TurnRight_Power(0.1);

            // Report yaw orientation to Driver Station.
            telemetry.addData("Robot overturned, Target Yaw value", angle);
            telemetry.addData("Current Yaw value", Yaw_Angle);
            telemetry.update();
        }


        Stop_Encoder();
    }


    // Encoders Only
    public void DriveForward_Encoder(int Distance, double speed) {
        Encoder_Distance = (int)(Distance*WHEEL_PULSES_PER_INCH);

        BackLeft.setTargetPosition(Encoder_Distance);
        BackRight.setTargetPosition(Encoder_Distance);

        BackLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        DriveForward_Power(speed);

        while (BackLeft.isBusy() || BackRight.isBusy()) {
            telemetry.addData("Driving Forward, Target Position",
                    Encoder_Distance);
            telemetry.addData("Driving Forward, Encoder Pulses Left",
                    BackLeft.getCurrentPosition());
            telemetry.addData("Driving Forward, Encoder Pulses Right",
                    BackRight.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addData("Reached Target Position","Encoder Pulses Left, Right",
                BackLeft.getCurrentPosition(), BackRight.getCurrentPosition());
        telemetry.update();

        Stop_Encoder();
    }
    public void DriveBackward_Encoder(int Distance, double speed){
        Encoder_Distance = (int)(Distance*WHEEL_PULSES_PER_INCH);

        BackLeft.setTargetPosition(Encoder_Distance);
        BackRight.setTargetPosition(Encoder_Distance);

        BackLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        DriveBackward_Power(-speed);

        while (BackLeft.isBusy() || BackRight.isBusy()) {
            telemetry.addData("Driving Backward, Target Position",
                    Encoder_Distance);
            telemetry.addData("Driving Backward, Encoder Pulses Left",
                    BackLeft.getCurrentPosition());
            telemetry.addData("Driving Backward, Encoder Pulses Right",
                    BackRight.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addData("Reached Target Position","Encoder Pulses Left, Right",
                BackLeft.getCurrentPosition(), BackRight.getCurrentPosition());
        telemetry.update();

        Stop_Encoder();
    }
    public void Stop_Encoder(){
        BackLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Robot stopped. Encoder Pulses (Cart)",
                        BackLeft.getCurrentPosition());
        telemetry.update();
    }


    // Power only
    public void DriveForward_Power(double speed) {
        leftPower = speed;
        rightPower = speed;

        FrontLeft.setPower(leftPower);
        BackLeft.setPower(leftPower/ BACK_WHEEL_POWER_REDUCTION);
        FrontRight.setPower(rightPower);
        BackRight.setPower(rightPower/ BACK_WHEEL_POWER_REDUCTION);
    }
    public void DriveBackward_Power(double speed) {
        leftPower = speed;
        rightPower = speed;

        FrontLeft.setPower(-leftPower);
        BackLeft.setPower(-leftPower/ BACK_WHEEL_POWER_REDUCTION);
        FrontRight.setPower(-rightPower);
        BackRight.setPower(-rightPower/ BACK_WHEEL_POWER_REDUCTION);
    }
    public void TurnLeft_Power(double speed) {
        leftPower = speed;
        rightPower = speed;

        FrontLeft.setPower(-leftPower);
        BackLeft.setPower(-leftPower/ BACK_WHEEL_POWER_REDUCTION);
        FrontRight.setPower(rightPower);
        BackRight.setPower(rightPower/ BACK_WHEEL_POWER_REDUCTION);
    }
    public void TurnRight_Power(double speed) {
        leftPower = speed;
        rightPower = speed;

        FrontLeft.setPower(leftPower);
        BackLeft.setPower(leftPower/ BACK_WHEEL_POWER_REDUCTION);
        FrontRight.setPower(-rightPower);
        BackRight.setPower(-rightPower/ BACK_WHEEL_POWER_REDUCTION);
    }
    public void Stop_Power() {
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);
    }


    //----------------------------------------------------------------------------------------------
    // Mechanism Functions
    //----------------------------------------------------------------------------------------------

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
    public void RotateArm(double power) {
        ArmRotator.setPower(power);
    }

    public void ArmCollecting() {
        ArmRotator.setTargetPosition(ARM_COLLECTING_ENCODER_PULSES);
        ArmRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        ArmRotator.setPower(0.5);

        if (ArmRotator.isBusy()) {
            telemetry.addData("Arm Rotating, Target Position",
                    ARM_COLLECTING_ENCODER_PULSES);
            telemetry.addData("Arm Encoder Pulses",
                    ArmRotator.getCurrentPosition());
            telemetry.update();
        }

        if (ArmRotator.getCurrentPosition() < 50) {
            telemetry.addData("Reached Collecting Position, Arm Encoder Pulses",
                    ArmRotator.getCurrentPosition());
            telemetry.update();

            ArmRotator.setPower(0);
        }
    }
    public void ArmBottomLevel() {
        ArmRotator.setTargetPosition(ARM_BOTTOM_LEVEL_ENCODER_PULSES);
        ArmRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        ArmRotator.setPower(0.5);

        if (ArmRotator.isBusy()) {
            telemetry.addData("Arm Rotating, Target Position",
                    ARM_BOTTOM_LEVEL_ENCODER_PULSES);
            telemetry.addData("Arm Encoder Pulses",
                    ArmRotator.getCurrentPosition());
            telemetry.update();
        }
    }
    public void ArmMidLevel() {
        ArmRotator.setTargetPosition(ARM_MID_LEVEL_ENCODER_PULSES);
        ArmRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        ArmRotator.setPower(0.5);

        if (ArmRotator.isBusy()) {
            telemetry.addData("Arm Rotating, Target Position",
                    ARM_MID_LEVEL_ENCODER_PULSES);
            telemetry.addData("Arm Encoder Pulses",
                    ArmRotator.getCurrentPosition());
            telemetry.update();
        }
    }
    public void ArmTopLevel() {
        ArmRotator.setTargetPosition(ARM_TOP_LEVEL_ENCODER_PULSES);
        ArmRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        ArmRotator.setPower(0.5);

        if (ArmRotator.isBusy()) {
            telemetry.addData("Arm Rotating, Target Position",
                    ARM_TOP_LEVEL_ENCODER_PULSES);
            telemetry.addData("Arm Encoder Pulses",
                    ArmRotator.getCurrentPosition());
            telemetry.update();
        }
    }

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
