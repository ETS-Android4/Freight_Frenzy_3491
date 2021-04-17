package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robots.Beyonce;
import org.firstinspires.ftc.teamcode.newhardware.Motor;
import org.firstinspires.ftc.teamcode.opmodesupport.TeleOpMode;


@TeleOp
public class AprilBeyonceTeleOp extends TeleOpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private DcMotor shooter = null;
    private DcMotor arm = null;
    private Motor feeder;
    private Motor ziptiePuller;
    //public DcMotor Led;

    private CRServo beatInStick;
    private Servo claw;
    private Servo ramp = null;
    private Servo wallHolder;
    private Servo ringPusher;


    public void initialize() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontL");
        frontRight = hardwareMap.get(DcMotor.class, "frontR");
        backLeft = hardwareMap.get(DcMotor.class, "backL");
        backRight = hardwareMap.get(DcMotor.class, "backR");

        shooter = hardwareMap.get(DcMotor.class, "Shooter");
        arm = hardwareMap.get(DcMotor.class, "Arm");
        ziptiePuller = new Motor("ziptiepuller");
        feeder = new Motor("feeder");
        beatInStick = hardwareMap.get(CRServo.class, "BeatinStick");

        ramp = hardwareMap.get(Servo.class, "Ramp");
        wallHolder = hardwareMap.get(Servo.class, "WallHolder");
        ringPusher = hardwareMap.get (Servo.class, "RingPusher");
        claw = hardwareMap.get(Servo.class, "Claw");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Indicate that the program is running
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    //Gives user 2 drive control options
    boolean leftHanded = true;

    //Declaring drive variables
    double horizontal;
    double vertical;
    double pivot;



//nice

    public void loopOpMode() {
        //Drive control option one
        if (leftHanded) {
            pivot = gamepad1.left_stick_x;
            horizontal = -gamepad1.right_stick_x;
            vertical = gamepad1.right_stick_y;

            if (gamepad1.dpad_right) {
                leftHanded = false;
            }

        //Drive control option two
        } else {
            pivot = gamepad1.right_stick_x;
            horizontal = -gamepad1.left_stick_x;
            vertical = gamepad1.left_stick_y;

            if (gamepad1.dpad_left) {
                leftHanded = true;
            }
        }

        //Drive calculations

        if (gamepad1.right_trigger > 0) {
            if (gamepad1.left_trigger > 0) {
                frontLeft.setPower((-pivot + (vertical + horizontal)) / -2);
                frontRight.setPower((pivot + (vertical - horizontal)) / -2);
                backLeft.setPower((-pivot + (vertical - horizontal)) / -2);
                backRight.setPower((pivot + (vertical + horizontal)) / -2);
                telemetry.addData("s", "slow");
            } else {
                frontLeft.setPower((pivot + (vertical + horizontal)) / 2);
                frontRight.setPower((-pivot + (vertical - horizontal)) / 2);
                backLeft.setPower((pivot + (vertical - horizontal)) / 2);
                backRight.setPower((-pivot + (vertical + horizontal)) / 2);
                telemetry.addData("s", "slow");
            }
        } else {
            if (gamepad1.left_trigger > 0) {
                frontLeft.setPower((-pivot + (vertical + horizontal)) * -1);
                frontRight.setPower((pivot + (vertical - horizontal)) * -1);
                backLeft.setPower((-pivot + (vertical - horizontal)) * -1);
                backRight.setPower((pivot + (vertical + horizontal)) * -1);
                telemetry.addData("s", "fast");
            } else {
                frontLeft.setPower((pivot + (vertical + horizontal)));
                frontRight.setPower((-pivot + (vertical - horizontal)));
                backLeft.setPower((pivot + (vertical - horizontal)));
                backRight.setPower((-pivot + (vertical + horizontal)));
                telemetry.addData("s", "fast");
            }
        }


        //Wobble grabber Arm
        moveArm(gamepad2.right_stick_y);

        //Wobble grabber Claw
        if (gamepad2.b) {
            clawClose();
            telemetry.addData("Claw ", "closing");
        } else if (gamepad2.x) {
            clawOpen();
            telemetry.addData("claw", "open");
        }

        beat(gamepad2.left_stick_x);


        if (gamepad2.left_trigger > 0){
            holdWall();
            telemetry.addData("wall", "hold");
        }
        else if (gamepad2.left_bumper){
            releaseWall();
            telemetry.addData("wall", "release");
        }



//        if (gamepad2.y) {
//            beyonce.Led.setPower(0);
//        }
//        beyonce.Led.setPower(1);

        //Ring pusher
        if (gamepad2.right_trigger > 0) {
            ringPusherExtend();
        } else {
            ringPusherRetract();
        }

        moveRamp(gamepad2.dpad_up, gamepad2.dpad_down);

        //Shooter
        if (gamepad2.dpad_right) {
            shooterOn();
        }
        if (gamepad2.dpad_left) {
            shooterOff();
        }

        if (gamepad1.left_bumper){
            ramp.setPosition(0.8);
        }
        if (gamepad1.right_bumper){
            ramp.setPosition(0.2);
        }


        if (gamepad2.a){
            feeder.setPower(-1);

            ziptiePuller.setPower(0.5);
        }

        if (gamepad2.y){
            feeder.setPower(0);

            ziptiePuller.setPower(0);
        }

        if (gamepad2.left_stick_button) {
            feeder.setPower(1);

            ziptiePuller.setPower(-0.5);
        }



        //Keeps user updated
        telemetry.addData("arm:", (gamepad2.right_stick_y/4));
        telemetry.update();

    }

    private void moveArm(double joystickValue) {
        ((DcMotorEx)arm).setVelocity(joystickValue * 1440);
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

    private void holdWall(){
        wallHolder.setPosition(0.2);
    }
    private void releaseWall(){
        wallHolder.setPosition(0.8);
    }
}