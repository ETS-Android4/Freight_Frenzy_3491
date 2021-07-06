package org.firstinspires.ftc.teamcode.gamecode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robots.Beyonce;

@Disabled
@TeleOp
public class BeyonceTeleOp extends LinearOpMode {

    //probably going to be useful. if it aint broke, dont fix it
    private ElapsedTime runtime = new ElapsedTime();

    //motors
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor linearSlide = null;
    private DcMotor shooter = null;

    //servos
    private Servo grabber = null;
    //private Servo targetRamp = null;
    private Servo ringPusher = null;

    public boolean ShooterOn = false;


    public void runOpMode() {
        //indicate that the program is running
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //find moters in the configuration file on phones
        frontLeft = hardwareMap.get(DcMotor.class, "frontL");
        frontRight = hardwareMap.get(DcMotor.class, "frontR");
        backLeft = hardwareMap.get(DcMotor.class, "backL");
        backRight = hardwareMap.get(DcMotor.class, "backR");
        linearSlide = hardwareMap.get(DcMotor.class, "Linear Slide"); //done
        shooter = hardwareMap.get(DcMotor.class, "Shooter");

        grabber = hardwareMap.get(Servo.class, "Grabber"); //done
        //targetRamp = hardwareMap.get(Servo.class, "TargetRamp"); //done
        ringPusher = hardwareMap.get(Servo.class, "RingPusher"); //done


        //set all motors to drive in the same direction in the real world
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        //while opMode is active do the stuff in the while loop
        while (opModeIsActive()) {

            //variables to control mechanisms
/*nice*/    int target = 1;

            //so that i dont have to write gamepad1.left_stick every single time for the calculations
            double horizontal = gamepad1.left_stick_x;
            double vertical = -gamepad1.left_stick_y;
            double pivot = gamepad1.right_stick_x;

            //drive controls/calculations
            frontLeft.setPower(pivot + (vertical + horizontal));
            frontRight.setPower(-pivot + (vertical - horizontal));
            backLeft.setPower(pivot + (vertical - horizontal));
            backRight.setPower(-pivot + (vertical + horizontal));

            //controls to move the linear slide up and down
            if (gamepad2.y == true){
                linearSlide.setPower(1);
            } else if (gamepad2.a = true){
                linearSlide.setPower(-1);
            } else {
                linearSlide.setPower(0);
            }

            //controls to move the grabber mechanism.
            //NOTE: Positions may need adjusting. change/adjust the number in the brackets of grabber.setPosition(1); and/or grabber.setPosition(0); so that the servo goes to the correct position
            if (gamepad2.b == true) {
                grabber.setPosition(1);
            } else {
                grabber.setPosition(0);
            }

            //controls for the ring pusher
            //NOTE: Positions may need adjusting. change/adjust the number in the brackets of ringPusher.setPosition(1); and/or ringPusher.setPosition(0); so that the servo goes to the correct position
            if (gamepad2.right_trigger > 0){
                ringPusher.setPosition(1);
                ringPusher.setPosition(0);
            } else {
                ringPusher.setPosition(0);
            }

            if (gamepad2.dpad_up == true) {
                if (target == 1) {
            //        targetRamp.setPosition(0);
                    target = 2;
                }
                else if (target == 2) {
             //       targetRamp.setPosition(0.5);
                    target = 3;
                }
                else if (target == 3) {
            //        targetRamp.setPosition(1);
                    target = 1;
                }
            }

            //failsafe for the shooter
//            if (gamepad2.dpad_right = true) {
//                boolean failSafe = true;
//                if (failSafe = true){
//                    shooter.setPower(0);
//                    if (gamepad2.dpad_right = true){
//                        failSafe = false;
//                    }
//                }
//            }

            //shooter.setPower(1);



            if (gamepad2.dpad_right == true){ //if the button is pressed
                if (gamepad2.dpad_right == false){ //if the button was not pressed

                    if (ShooterOn == true){
                        ShooterOn = false;
                    }

                    if (ShooterOn == false) {
                        ShooterOn = true;
                    }
                }
            }

            if (ShooterOn){

            }



            //keeps user updated
            telemetry.addData("Motors", "horizontal (%.2f), vertical (%.2f), pivot (%.2f)", horizontal, vertical, pivot);
            telemetry.update();



        }
    }
}
