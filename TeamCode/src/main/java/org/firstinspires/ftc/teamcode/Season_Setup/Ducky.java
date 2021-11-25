// Hardware and Autonomous set-up for Robot for 2021-2022 Freight Frenzy

package org.firstinspires.ftc.teamcode.Season_Setup;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    // Declaring opMode Variables
    HardwareMap hwMap;
    Telemetry telemetry;

    // Checking runtime of functions
    public ElapsedTime runtime = new ElapsedTime();

    // Declaring Drivebase Motor Variables
    public DcMotorEx FrontLeft, BackLeft, FrontRight, BackRight;

    // Declaring Mechanisms
    public DcMotorEx ArmRotator;
    public CRServo Collector;

    public CRServo CarouselSpinner;


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

    public boolean resetArm;

    // EasyOpenCV Setup
    public OpenCvCamera webcam;
    public static double analysisLeft = 0.0;
    public static double analysisCenter = 0.0;
    public static double analysisRight = 0.0;

    // IMU functions
    public Orientation angles;
    public Acceleration gravity;
    public static final double TURN_ANGLE_TOLERANCE = 2;

    // Autonomous state
    public boolean leftState;
    public boolean centerState;
    public boolean rightState;


    // Alliance Determination
    public static String alliance;


    // PID
    public double Kp = 0.03;
    public double Kd = 0.00005;

    // Class Constructor
    public Ducky(){

    }

    public void init(HardwareMap ahwMap, Telemetry a_telemetry)  {

        // Calling variable
        hwMap = ahwMap;
        telemetry = a_telemetry;

//        // Define Drive Motors
//        FrontLeft = hwMap.get(DcMotorEx.class,"frontL");
//        BackLeft = hwMap.get(DcMotorEx.class,"backL");
//        FrontRight = hwMap.get(DcMotorEx.class,"frontR");
//        BackRight = hwMap.get(DcMotorEx.class,"backR");
//
//        // Setting Motor Direction
//        FrontLeft.setDirection(DcMotorEx.Direction.REVERSE);
//        BackLeft.setDirection(DcMotorEx.Direction.REVERSE);
//        FrontRight.setDirection(DcMotorEx.Direction.FORWARD);
//        BackRight.setDirection(DcMotorEx.Direction.FORWARD);
//
//        // Setting Motor zero power Behaviour
//        FrontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        BackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        FrontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        BackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//
//        // Mechanisms - Motors
//        ArmRotator = hwMap.get(DcMotorEx.class,"armRotator");
//
//        // Mechanisms - Setting Motor Direction
//        FrontRight.setDirection(DcMotorEx.Direction.FORWARD);
//
//        // Mechanisms - Setting Motor zero power Behaviour
//        ArmRotator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//
//        // Define Servos
//        Collector = hwMap.crservo.get("collector");
//        CarouselSpinner = hwMap.crservo.get("carouselSpinner");
//
//        // Initialize Servos
//        Collector.setPower(0);
//        CarouselSpinner.setPower(0);
//
//
//        // Resetting Motor Encoders
//        BackLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        BackRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        ArmRotator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//
//        // Setting Motors to run with/ without Encoders - (RUN_WITHOUT_ENCODER/ RUN_USING_ENCODER)
//        BackLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        BackRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        ArmRotator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//
//        // IMU
//        imu = hwMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        imu.initialize(parameters);
//
//
//        // EasyOpenCV Setup
//        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webcam.setPipeline(new Freight_Frenzy_Pipeline.Pipeline());
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
/*    public void DriveForward_Encoder_IMU (int Distance, double speed) {
        Encoder_Distance = (int)(Distance* WHEEL_PULSES_PER_INCH);

        BackLeft.setTargetPosition(Encoder_Distance);
        BackRight.setTargetPosition(Encoder_Distance);

        BackLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        DriveForward_Power(speed);

        while (BackLeft.isBusy() || BackRight.isBusy()) {

            double currentHeading = getHeading();

            // If robot is drifting right - Need to turn left
            if (currentHeading < -5) {
                if (rightPower < speed+0.2){
                    leftPower -= 0.05;
                    rightPower += 0.05;
                }

                FrontLeft.setPower(leftPower);
                BackLeft.setPower(leftPower);
                FrontRight.setPower(rightPower);
                BackRight.setPower(rightPower);

            // If robot is drifting left - Need to turn right
            } else if (currentHeading > 5) {
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
            telemetry.addData("Yaw value", currentHeading);

            telemetry.update();
        }

        Stop_Encoder();
    }
    public void DriveBackward_Encoder_IMU (int Distance, double speed) {
        Encoder_Distance = (int)(-Distance* WHEEL_PULSES_PER_INCH);

        BackLeft.setTargetPosition(Encoder_Distance);
        BackRight.setTargetPosition(Encoder_Distance);

        BackLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        DriveBackward_Power(speed);

        while (BackLeft.isBusy() || BackRight.isBusy()) {

            double currentHeading = getHeading();

            // If robot is drifting right - Need to turn left
            if (currentHeading < -5) {
                if (leftPower < speed+0.2){
                    leftPower += 0.05;
                    rightPower -= 0.05;
                }

                FrontLeft.setPower(leftPower);
                BackLeft.setPower(leftPower);
                FrontRight.setPower(rightPower);
                BackRight.setPower(rightPower);

            // If robot is drifting left - Need to turn right
            } else if (currentHeading > 5) {
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
            telemetry.addData("Yaw value", currentHeading);

            telemetry.update();
        }

        Stop_Encoder();
    }*/

    // Turning with IMU and P control (PID without ID)
    public void turn_P(double turnDegree, double timeout, double initialSleep) throws InterruptedException {
        runtime.reset();

        Thread.sleep((long) initialSleep);

        turnDegree = turnDegree - (TURN_ANGLE_TOLERANCE/2);

        double currTime = runtime.seconds();
        double prevTime = currTime;
        double currentHeading = getHeading();
        double targetHeading = adjustHeading(currentHeading + turnDegree);
        double error = adjustHeading(targetHeading - currentHeading);
        double prevError = error;

        while (runtime.milliseconds() < timeout && Math.abs(error) > TURN_ANGLE_TOLERANCE)
        {
            double deltaTime = currTime - prevTime;
            double pTerm = Kp*error;
            double dTerm = deltaTime > 0.0? Kd*(error - prevError)/deltaTime: 0.0;

            prevTime = currTime;
            prevError = error;

            drivePower(0.0, clipRange(pTerm + dTerm, -1.0, 1.0));

            currTime = runtime.seconds();
            currentHeading = getHeading();
            error = adjustHeading(targetHeading - currentHeading);

            // Telemetry Update
            telemetry.addData("Target Yaw value", targetHeading);
            telemetry.addData("Current Yaw value", currentHeading);
            telemetry.addData("Error", error);
            telemetry.addData("Loop time", deltaTime);
            telemetry.update();
        }
        Stop_Power();
    }
    double adjustHeading(double heading) {
        return (heading <= -180.0)? heading + 360.0: (heading > 180.0)? heading - 360.0: heading;
    }
    double getHeading() {
        return -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
    }


    // Encoders Only
    public void DriveForward_Encoder(int Distance, double speed, double timeout) {
        runtime.reset();

        Encoder_Distance = (int)(Distance*WHEEL_PULSES_PER_INCH);

        BackLeft.setTargetPosition(Encoder_Distance);
        BackRight.setTargetPosition(Encoder_Distance);

        BackLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        speed = Math.abs(speed);
        DriveForward_Power(speed);

        while ((BackLeft.isBusy() || BackRight.isBusy()) && runtime.milliseconds() < timeout) {
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
    public void DriveBackward_Encoder(int Distance, double speed, double timeout){
        runtime.reset();

        Encoder_Distance = (int)(-Distance*WHEEL_PULSES_PER_INCH);

        BackLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        BackLeft.setTargetPosition(Encoder_Distance);
        BackRight.setTargetPosition(Encoder_Distance);

        BackLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        speed = Math.abs(speed);
        DriveBackward_Power(speed);

        while ((BackLeft.isBusy() || BackRight.isBusy()) && runtime.milliseconds() < timeout) {
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
        Stop_Power();

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
        BackLeft.setPower(leftPower* BACK_WHEEL_POWER_REDUCTION);
        FrontRight.setPower(rightPower);
        BackRight.setPower(rightPower* BACK_WHEEL_POWER_REDUCTION);
    }
    public void DriveBackward_Power(double speed) {
        leftPower = speed;
        rightPower = speed;

        FrontLeft.setPower(-leftPower);
        BackLeft.setPower(-leftPower* BACK_WHEEL_POWER_REDUCTION);
        FrontRight.setPower(-rightPower);
        BackRight.setPower(-rightPower* BACK_WHEEL_POWER_REDUCTION);
    }
    public void Stop_Power() {
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);
    }


    // Function that works for driving and turning
    public void drivePower(double drivePower, double turnPower) {
        leftPower = clipRange(drivePower + turnPower, -1.0, 1.0);
        rightPower = clipRange(drivePower - turnPower, -1.0, 1.0);

        FrontLeft.setPower(leftPower);
        BackLeft.setPower(leftPower* BACK_WHEEL_POWER_REDUCTION);
        FrontRight.setPower(rightPower);
        BackRight.setPower(rightPower* BACK_WHEEL_POWER_REDUCTION);
    }
    @SuppressWarnings("SameParameterValue")
    double clipRange(double value, double minValue, double maxValue)
    {
        return value < minValue? minValue: Math.min(value, maxValue);
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
            if (ArmRotator.getCurrentPosition() < 50) {
                telemetry.addData("Reached Collecting Position, Arm Encoder Pulses",
                        ArmRotator.getCurrentPosition());
                telemetry.update();

                ArmRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else {
                telemetry.addData("Arm Rotating, Target Position",
                        ARM_COLLECTING_ENCODER_PULSES);
                telemetry.addData("Arm Encoder Pulses",
                        ArmRotator.getCurrentPosition());
                telemetry.update();
            }
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
            ArmRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
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
    public void CarouselSpinnerBlue(){
        CarouselSpinner.setPower(1);
    }
    public void CarouselSpinnerRed(){
        CarouselSpinner.setPower(-1);
    }
    public void CarouselSpinnerOff(){
        CarouselSpinner.setPower(0);
    }


    //----------------------------------------------------------------------------------------------
    // Miscellaneous Functions
    //----------------------------------------------------------------------------------------------

    public void writeAndRead (String allianceColour) {
        File_WriteAndRead.writeToFile(allianceColour);
        File_WriteAndRead.readFromFile();
    }
}
