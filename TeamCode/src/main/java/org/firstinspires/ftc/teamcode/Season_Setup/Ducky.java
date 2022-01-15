// Hardware and Autonomous set-up for Robot for 2021-2022 Freight Frenzy

package org.firstinspires.ftc.teamcode.Season_Setup;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
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
    public DcMotorEx frontLeft, backLeft, frontRight, backRight;

    // Declaring Mechanisms
    public DcMotorEx armPlatformRotator;
    public DcMotorEx armRotator;
    public DcMotorEx armExtender;

    public CRServo collector;

    public DcMotorEx carouselSpinnerRotator;
    public CRServo carouselSpinner;

    public Servo tapeMeasure;

    // Declaring sensors
    public BNO055IMU imu;

    // Motor Power Variables
    public double leftPower;
    public double rightPower;

    // Encoder + Wheel Declaration
    public static final double BACK_WHEEL_POWER_REDUCTION = 0.9449;
    public static final double WHEEL_GEAR_RATIO = 2; // 2:1 ratio

    public static final double WHEEL_PULSES_PER_INCH = 34.2/ WHEEL_GEAR_RATIO; // Num of Pulses per inch travelled with Cart Wheel
    public int encoder_Distance; // To be used in functions for setTargetPosition

    public static final int ARM_COLLECTING_ENCODER_PULSES = 0;
    public static final int ARM_BOTTOM_LEVEL_ENCODER_PULSES = -150; //-94 //-182
    public static final int ARM_MID_LEVEL_ENCODER_PULSES = -420; //-230 //-351
    public static final int ARM_TOP_LEVEL_ENCODER_PULSES = -586; //-410 //-560

    public static final int ARM_PLATFORM_ROTATOR_MIDDLE = -60;
    public static final int ARM_PLATFORM_ROTATOR_HUB_FRONT = -573;
    public static final int ARM_PLATFORM_ROTATOR_HUB_BACK = 430;
    public static final int ARM_PLATFORM_ROTATOR_INITIAL = 0;

    public static final int DUCKY_SPINNER_ROTATOR_RED = 0;
    public static final int DUCKY_SPINNER_ROTATOR_BLUE = -757;
    public static final int DUCKY_SPINNER_ROTATOR_MIDDLE = -390;


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

        // Define Drive Motors
        frontLeft = hwMap.get(DcMotorEx.class,"frontL");
        backLeft = hwMap.get(DcMotorEx.class,"backL");
        frontRight = hwMap.get(DcMotorEx.class,"frontR");
        backRight = hwMap.get(DcMotorEx.class,"backR");

        // Setting Motor Direction
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        // Setting Motor zero power Behaviour
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        // Mechanisms - Motors
        armRotator = hwMap.get(DcMotorEx.class,"armRotator");
        armPlatformRotator = hwMap.get(DcMotorEx.class,"armPlatformRotator");
        carouselSpinnerRotator = hwMap.get(DcMotorEx.class,"carouselSpinnerRotator");
        armExtender = hwMap.get(DcMotorEx.class,"armExtender");



        // Mechanisms - Setting Motor Direction
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);

        // Mechanisms - Setting Motor zero power Behaviour
        armRotator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armPlatformRotator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        carouselSpinnerRotator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        // Define Servos
        collector = hwMap.crservo.get("collector");
        carouselSpinner = hwMap.crservo.get("carouselSpinner");
        tapeMeasure = hwMap.servo.get("tapeMeasure");


        // Initialize Servos
        collector.setPower(0);
        carouselSpinner.setPower(0);


        // Resetting Motor Encoders
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armRotator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armPlatformRotator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        carouselSpinnerRotator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Setting Motors to run with/ without Encoders - (RUN_WITHOUT_ENCODER/ RUN_USING_ENCODER)
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armRotator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armPlatformRotator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        carouselSpinnerRotator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        // IMU
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);


        // EasyOpenCV Setup
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new Freight_Frenzy_Pipeline.Pipeline());
    }



    //----------------------------------------------------------------------------------------------
    // Driving Functions
    //----------------------------------------------------------------------------------------------

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
        stop_Power();
    }
    double adjustHeading(double heading) {
        return (heading <= -180.0)? heading + 360.0: (heading > 180.0)? heading - 360.0: heading;
    }
    double getHeading() {
        return -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
    }


    // Encoders Only
    public void driveForward_Encoder(int Distance, double speed, double timeout) {
        runtime.reset();

        encoder_Distance = (int)(Distance*WHEEL_PULSES_PER_INCH);

        backLeft.setTargetPosition(encoder_Distance);
        backRight.setTargetPosition(encoder_Distance);

        backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        speed = Math.abs(speed);
        driveForward_Power(speed);

        while ((backLeft.isBusy() || backRight.isBusy()) && runtime.milliseconds() < timeout) {
            telemetry.addData("Driving Forward, Target Position",
                    encoder_Distance);
            telemetry.addData("Driving Forward, Encoder Pulses Left",
                    backLeft.getCurrentPosition());
            telemetry.addData("Driving Forward, Encoder Pulses Right",
                    backRight.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addData("Reached Target Position","Encoder Pulses Left, Right",
                backLeft.getCurrentPosition(), backRight.getCurrentPosition());
        telemetry.update();

        stop_Encoder();
    }
    public void driveBackward_Encoder(int Distance, double speed, double timeout){
        runtime.reset();

        encoder_Distance = (int)(-Distance*WHEEL_PULSES_PER_INCH);

        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setTargetPosition(encoder_Distance);
        backRight.setTargetPosition(encoder_Distance);

        backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        speed = Math.abs(speed);
        driveBackward_Power(speed);

        while ((backLeft.isBusy() || backRight.isBusy()) && runtime.milliseconds() < timeout) {
            telemetry.addData("Driving Backward, Target Position",
                    encoder_Distance);
            telemetry.addData("Driving Backward, Encoder Pulses Left",
                    backLeft.getCurrentPosition());
            telemetry.addData("Driving Backward, Encoder Pulses Right",
                    backRight.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addData("Reached Target Position","Encoder Pulses Left, Right",
                backLeft.getCurrentPosition(), backRight.getCurrentPosition());
        telemetry.update();

        stop_Encoder();
    }
    public void stop_Encoder(){
        stop_Power();

        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Robot stopped. Encoder Pulses (Cart)",
                        backLeft.getCurrentPosition());
        telemetry.update();
    }


    // Power only
    public void driveForward_Power(double speed) {
        leftPower = speed;
        rightPower = speed;

        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower* BACK_WHEEL_POWER_REDUCTION);
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower* BACK_WHEEL_POWER_REDUCTION);
    }
    public void driveBackward_Power(double speed) {
        leftPower = speed;
        rightPower = speed;

        frontLeft.setPower(-leftPower);
        backLeft.setPower(-leftPower* BACK_WHEEL_POWER_REDUCTION);
        frontRight.setPower(-rightPower);
        backRight.setPower(-rightPower* BACK_WHEEL_POWER_REDUCTION);
    }
    public void stop_Power() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }


    // Function that works for driving and turning
    public void drivePower(double drivePower, double turnPower) {
        leftPower = clipRange(drivePower + turnPower, -1.0, 1.0);
        rightPower = clipRange(drivePower - turnPower, -1.0, 1.0);

        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower* BACK_WHEEL_POWER_REDUCTION);
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower* BACK_WHEEL_POWER_REDUCTION);
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
    public void collectorOn(){
        collector.setPower(1);
    }
    public void collectorReverse(){
        collector.setPower(-1);
    }
    public void collectorOff(){
        collector.setPower(0);
    }

    // Arm Rotator
    public void rotateArm(double power) {
        armRotator.setPower(power);
    }

    public void armCollecting() {
        armRotator.setTargetPosition(ARM_COLLECTING_ENCODER_PULSES);
        armRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        armRotator.setPower(0.5);

        if (armRotator.isBusy()) {
            if (armRotator.getCurrentPosition() < 50) {
                telemetry.addData("Reached Collecting Position, Arm Encoder Pulses",
                        armRotator.getCurrentPosition());
                telemetry.update();

                armRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else {
                telemetry.addData("Arm Rotating, Target Position",
                        ARM_COLLECTING_ENCODER_PULSES);
                telemetry.addData("Arm Encoder Pulses",
                        armRotator.getCurrentPosition());
                telemetry.update();
            }
        }
    }
    public void armBottomLevel() {
        armRotator.setTargetPosition(ARM_BOTTOM_LEVEL_ENCODER_PULSES);
        armRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        armRotator.setPower(0.5);

        if (armRotator.isBusy()) {
            telemetry.addData("Arm Rotating, Target Position",
                    ARM_BOTTOM_LEVEL_ENCODER_PULSES);
            telemetry.addData("Arm Encoder Pulses",
                    armRotator.getCurrentPosition());
            telemetry.update();
        }
    }
    public void armMidLevel() {
        armRotator.setTargetPosition(ARM_MID_LEVEL_ENCODER_PULSES);
        armRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        armRotator.setPower(0.5);

        if (armRotator.isBusy()) {
            armRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            telemetry.addData("Arm Rotating, Target Position",
                    ARM_MID_LEVEL_ENCODER_PULSES);
            telemetry.addData("Arm Encoder Pulses",
                    armRotator.getCurrentPosition());
            telemetry.update();
        }
    }
    public void armTopLevel() {
        armRotator.setTargetPosition(ARM_TOP_LEVEL_ENCODER_PULSES);
        armRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        armRotator.setPower(0.5);

        if (armRotator.isBusy()) {
            telemetry.addData("Arm Rotating, Target Position",
                    ARM_TOP_LEVEL_ENCODER_PULSES);
            telemetry.addData("Arm Encoder Pulses",
                    armRotator.getCurrentPosition());
            telemetry.update();
        }
    }
//    void armHoldAtPosition(double pos, boolean inDegrees)
//    {
//        double targetDegrees = inDegrees? pos: pos/ARM_ENCODER_TICKS_PER_DEGREE + ARM_MIN_POS_IN_DEGREES;
//        int targetEncoderTicks = (int)(inDegrees? (pos - ARM_MIN_POS_IN_DEGREES)*ARM_ENCODER_TICKS_PER_DEGREE: pos);
//        int error = targetEncoderTicks - armRotator.getCurrentPosition();
//        double gravityCompensation = ARM_GC_POWER*Math.cos(Math.toRadians(targetDegrees - 90));
//        double armPower = ARM_KP*error + gravityCompensation;
//        armRotator.setPower(armPower);
//    }

    // Arm Platform Rotator
    public void rotateArmPlatform(double power) {
        armPlatformRotator.setPower(power);
    }

    public void armPlatform_Hub_Right() {
        armPlatformRotator.setTargetPosition(ARM_PLATFORM_ROTATOR_HUB_FRONT);
        armPlatformRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        armPlatformRotator.setPower(0.5);

        if (armPlatformRotator.isBusy()) {
            telemetry.addData("Arm Platform Rotating, Target Position",
                    ARM_PLATFORM_ROTATOR_HUB_FRONT);
            telemetry.addData("Arm Platform Encoder Pulses",
                    armPlatformRotator.getCurrentPosition());
            telemetry.update();
        }
    }
    public void armPlatform_Hub_Left() {
        armPlatformRotator.setTargetPosition(ARM_PLATFORM_ROTATOR_HUB_BACK);
        armPlatformRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        armPlatformRotator.setPower(0.5);

        if (armPlatformRotator.isBusy()) {
            telemetry.addData("Arm Platform Rotating, Target Position",
                    ARM_PLATFORM_ROTATOR_HUB_BACK);
            telemetry.addData("Arm Platform Encoder Pulses",
                    armPlatformRotator.getCurrentPosition());
            telemetry.update();
        }
    }

    // Arm Extender
    public void extendArm(){
        armExtender.setPower(0.5);
    }
    public void retractArm(){
        armExtender.setPower(-0.5);
    }
    public void armExtenderOff(){
        armExtender.setPower(0);
    }

    // Carousel Spinner
    public void carouselSpinnerBlue(){
        carouselSpinner.setPower(1);
    }
    public void carouselSpinnerRed(){
        carouselSpinner.setPower(-1);
    }
    public void carouselSpinnerOff(){
        carouselSpinner.setPower(0);
    }

    // Ducky Spinner Rotator
    public void rotateToBlue(){
        carouselSpinnerRotator.setTargetPosition(DUCKY_SPINNER_ROTATOR_BLUE);
        carouselSpinnerRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        carouselSpinnerRotator.setPower(0.5);

        if (carouselSpinnerRotator.isBusy()) {
            telemetry.addData("Ducky Spinner Platform Rotating, Target Position",
                    DUCKY_SPINNER_ROTATOR_BLUE);
            telemetry.addData("Ducky Spinner Rotator Encoder Pulses",
                    carouselSpinnerRotator.getCurrentPosition());
            telemetry.update();
        }
    }
    public void rotateToRed(){
        carouselSpinnerRotator.setTargetPosition(DUCKY_SPINNER_ROTATOR_RED);
        carouselSpinnerRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        carouselSpinnerRotator.setPower(0.5);

        if (carouselSpinnerRotator.isBusy()) {
            telemetry.addData("Ducky Spinner Platform Rotating, Target Position",
                    DUCKY_SPINNER_ROTATOR_RED);
            telemetry.addData("Ducky Spinner Rotator Encoder Pulses",
                    carouselSpinnerRotator.getCurrentPosition());
            telemetry.update();
        }
    }
    public void rotateToMiddle(){
        carouselSpinnerRotator.setTargetPosition(DUCKY_SPINNER_ROTATOR_MIDDLE);
        carouselSpinnerRotator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        carouselSpinnerRotator.setPower(0.5);

        if (carouselSpinnerRotator.isBusy()) {
            telemetry.addData("Ducky Spinner Platform Rotating, Target Position",
                    DUCKY_SPINNER_ROTATOR_MIDDLE);
            telemetry.addData("Ducky Spinner Rotator Encoder Pulses",
                    carouselSpinnerRotator.getCurrentPosition());
            telemetry.update();
        }
    }

    // Tape Measure
    public void extendTapeMeasure(){
        tapeMeasure.setPosition(0.8);
    }
    public void retractTapeMeasure(){
        tapeMeasure.setPosition(0.2);
    }


    //----------------------------------------------------------------------------------------------
    // Miscellaneous Functions
    //----------------------------------------------------------------------------------------------

    public void writeAndRead (String allianceColour) {
        File_WriteAndRead.writeToFile(allianceColour);
        File_WriteAndRead.readFromFile();
    }

    public void enableWebcam () {
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public void disableWebcam () {
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }
}
