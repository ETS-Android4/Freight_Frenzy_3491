package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robots.Beyonce;


@TeleOp
public class JanuaryBeyonceTeleOp extends LinearOpMode {

    Beyonce beyonce = new Beyonce();

    //probably going to be useful. if it aint broke, dont fix it
    private ElapsedTime runtime = new ElapsedTime();

    //motors
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor arm = null;
    private DcMotor shooter = null;
//
//
//    private boolean buttonpressed = false;
//
//    public boolean ShooterOn = false;
//
//    //servos
//    private Servo claw = null;
    private CRServo targetRamp = null;
//    private Servo ringPusher = null;




    public void runOpMode() {
        //indicate that the program is running
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //find motors in the configuration file on phones
        frontLeft = hardwareMap.get(DcMotor.class, "frontL");
        frontRight = hardwareMap.get(DcMotor.class, "frontR");
        backLeft = hardwareMap.get(DcMotor.class, "backL");
        backRight = hardwareMap.get(DcMotor.class, "backR");
        arm = hardwareMap.get(DcMotor.class, "Arm");
        shooter = hardwareMap.get(DcMotor.class, "Shooter");

        targetRamp = hardwareMap.get(CRServo.class, "Ramp");


        //set all motors to drive in the same direction in the real world
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        //gives user 2 drive control options
//nice
        boolean leftHanded = true;

        //creates 3 vertical aiming options
        int target = 1; //for the target ramp

        boolean shooterToggle = false;
        int toggleTimer = 0;

        waitForStart();
        runtime.reset();


        //while opMode is active do the stuff in the while loop
        while (opModeIsActive()) {

            //so that i dont have to write gamepad1.left_stick every single time for the calculations
            double horizontal;
            double vertical;
            double pivot;


            if (leftHanded) {
                if (gamepad1.left_bumper) {
                    pivot = gamepad1.left_stick_x / 3;
                    horizontal = -gamepad1.right_stick_x / 3;
                    vertical = gamepad1.right_stick_y / 3;
                } else {
                    pivot = gamepad1.left_stick_x;
                    horizontal = -gamepad1.right_stick_x;
                    vertical = gamepad1.right_stick_y;
                }

                if (gamepad1.dpad_right) {
                    leftHanded = false;
                }
            } else {
                if (gamepad1.right_trigger > 0) {
                    pivot = gamepad1.right_stick_x / 3;
                    horizontal = -gamepad1.left_stick_x / 3;
                    vertical = gamepad1.left_stick_y / 3;
                } else {
                    pivot = gamepad1.right_stick_x;
                    horizontal = -gamepad1.left_stick_x;
                    vertical = gamepad1.left_stick_y;
                }

                if (gamepad1.dpad_left) {
                    leftHanded = true;
                }
            }

            //drive calculations
            frontLeft.setPower(pivot + (vertical + horizontal));
            frontRight.setPower(-pivot + (vertical - horizontal));
            backLeft.setPower(pivot + (vertical - horizontal));
            backRight.setPower(-pivot + (vertical + horizontal));

//            if (target < 4 && target > 0) {
//                if (gamepad2.dpad_up) {
//                    target++;
//                } else if (gamepad2.dpad_down) {
//                    target--;
//                }
//            }
//            //sets ramp level to target
//            //beyonce.setRampLevel(target);
//
//            targetRamp.setPower(gamepad2.left_stick_y);
//
//            arm.setPower(gamepad2.right_stick_y / 2);
//
//            //Wobble Grabber
//            if (gamepad2.x) {
//                beyonce.GrabberUp();
//            } else if (gamepad2.b){
//                beyonce.GrabberDown();
//            }
//
//            //ring pusher
//            if (gamepad2.right_trigger > 0) {
//                beyonce.RingPusherExtend();
//            } else {
//                beyonce.RingPusherRetract();
//            }
//
//            if (gamepad2.dpad_right) {
//                shooter.setPower(1);
//            }
//
//            if (gamepad2.dpad_left) {
//                shooter.setPower(0);
//            }


            //keeps user updated
            telemetry.addData("Motors", "horizontal (%.2f), vertical (%.2f), pivot (%.2f)", horizontal, vertical, pivot);
            telemetry.update();



        }
    }


}
