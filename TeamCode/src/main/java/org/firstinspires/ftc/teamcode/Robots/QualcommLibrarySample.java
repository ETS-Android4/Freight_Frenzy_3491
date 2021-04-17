package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.newhardware.FXTCRServo;
import org.firstinspires.ftc.teamcode.newhardware.FXTServo;
import org.firstinspires.ftc.teamcode.newhardware.Motor;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;

import static android.os.SystemClock.sleep;

@Autonomous
public class QualcommLibrarySample extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private DcMotor shooter = null;
    private DcMotor arm = null;
    private DcMotor feeder;
    private DcMotor ziptiePuller;
    //public DcMotor Led;

    private CRServo beatInStick;
    private Servo claw;
    private Servo ramp = null;
    private Servo wallHolder;
    private Servo ringPusher;

    ColorSensor colorSensorL;
    ColorSensor colorSensorR;


    double shooterPower = 0.0;  // not used in this example
    double shooterVelocity = 0.0;  // used to read velocity in this example

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        frontLeft = hardwareMap.get(DcMotor.class, "frontL");
        frontRight = hardwareMap.get(DcMotor.class, "frontR");
        backLeft = hardwareMap.get(DcMotor.class, "backL");
        backRight = hardwareMap.get(DcMotor.class, "backR");

        shooter = hardwareMap.get(DcMotor.class, "Shooter");
        arm = hardwareMap.get(DcMotor.class, "Arm");
        ziptiePuller = hardwareMap.get(DcMotor.class, "ziptiepuller");
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        beatInStick = hardwareMap.get(CRServo.class, "BeatinStick");

        ramp = hardwareMap.get(Servo.class, "Ramp");
        wallHolder = hardwareMap.get(Servo.class, "WallHolder");
        ringPusher = hardwareMap.get (Servo.class, "RingPusher");
        claw = hardwareMap.get(Servo.class, "Claw");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Resetting Encoder");
        telemetry.update();

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ((DcMotorEx) shooter).setVelocity(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // STOP was pressed
        ((DcMotorEx) shooter).setVelocity(0);
    }

    private void driveForward(double speed){
        ((DcMotorEx)frontLeft).setVelocity(speed);
        ((DcMotorEx)frontRight).setVelocity(speed);
        ((DcMotorEx)backLeft).setVelocity(speed);
        ((DcMotorEx)backRight).setVelocity(speed);
    }

    private void driveBackwards(double speed){
        ((DcMotorEx)frontLeft).setVelocity(-speed);
        ((DcMotorEx)frontRight).setVelocity(-speed);
        ((DcMotorEx)backLeft).setVelocity(-speed);
        ((DcMotorEx)backRight).setVelocity(-speed);
    }

    private void strafeLeft(double speed){
        ((DcMotorEx)frontLeft).setVelocity(-speed);
        ((DcMotorEx)frontRight).setVelocity(speed);
        ((DcMotorEx)backLeft).setVelocity(speed);
        ((DcMotorEx)backRight).setVelocity(-speed);
    }

    private void strafeRight(double speed){
        ((DcMotorEx)frontLeft).setVelocity(speed);
        ((DcMotorEx)frontRight).setVelocity(-speed);
        ((DcMotorEx)backLeft).setVelocity(-speed);
        ((DcMotorEx)backRight).setVelocity(speed);
    }

    private void turnRight(double speed){
        ((DcMotorEx)frontLeft).setVelocity(speed);
        ((DcMotorEx)frontRight).setVelocity(-speed);
        ((DcMotorEx)backLeft).setVelocity(speed);
        ((DcMotorEx)backRight).setVelocity(-speed);
    }

    private void turnLeft(double speed){
        ((DcMotorEx)frontLeft).setVelocity(-speed);
        ((DcMotorEx)frontRight).setVelocity(speed);
        ((DcMotorEx)backLeft).setVelocity(-speed);
        ((DcMotorEx)backRight).setVelocity(speed);
    }

    private void stopMotors() {
        ((DcMotorEx)frontLeft).setVelocity(0);
        ((DcMotorEx)frontRight).setVelocity(0);
        ((DcMotorEx)backLeft).setVelocity(0);
        ((DcMotorEx)backRight).setVelocity(0);
    }

    private void moveArm(double joystickValue) {
        ((DcMotorEx)arm).setVelocity(joystickValue * 1440);
    }

    public void armDown(double power) {
        ((DcMotorEx)arm).setVelocity(power * 1440);
    }

    private void beat(double power) {
        beatInStick.setPower(power);
    }

    private void clawOpen() {
        claw.setPosition(0);
    }
    private void clawClose() {
        claw.setPosition(1);
    }

    private void shooterOn() {
        ((DcMotorEx)shooter).setVelocity(2800);
    }
    private void shooterOff() {
        ((DcMotorEx)shooter).setVelocity(0);
    }

    private double position = 0;
    private void moveRamp(boolean dPadUp, boolean dPadDown) {
        if (dPadUp && position < 0.8) {
            position = position + 0.001;
        } else if (dPadDown && position > 0.2) {
            position = position - 0.001;
        }
        ramp.setPosition(position);
    }

    private void armAuto(double power) {
        arm.setPower(power);
    }

    private void ringPusherExtend() {
        ringPusher.setPosition(1);
    }
    private void ringPusherRetract() {
        ringPusher.setPosition(0.45);
    }

    private void shoot() {
        ringPusherExtend();
        sleep(1250);
        ringPusherRetract();
        sleep(750);
        sleep(2500);
    }

    private void holdWall(){
        wallHolder.setPosition(0.2);
    }
    private void releaseWall(){
        wallHolder.setPosition(0.8);
    }

}
