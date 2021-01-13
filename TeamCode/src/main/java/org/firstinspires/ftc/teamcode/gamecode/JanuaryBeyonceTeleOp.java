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
    
    
    
    public void runOpMode() {
        //indicate that the program is running
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //gives user 2 drive control options
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
            beyonce.FrontLeft.setPower(pivot + (vertical + horizontal));
            beyonce.FrontRight.setPower(-pivot + (vertical - horizontal));
            beyonce.BackLeft.setPower(pivot + (vertical - horizontal));
            beyonce.BackRight.setPower(-pivot + (vertical + horizontal));

            if (target < 4 && target > 0) {
                if (gamepad2.dpad_up) {
                    target++;
                } else if (gamepad2.dpad_down) {
                    target--;
                }
            }
            //sets ramp level to target
            //beyonce.setRampLevel(target);

            beyonce.Ramp.setPower(gamepad2.left_stick_y);

            beyonce.Arm.setPower(gamepad2.right_stick_y / 2);

            //Wobble Grabber
            if (gamepad2.x) {
                beyonce.ClawOpen();
            } else if (gamepad2.b){
                beyonce.ClawClose();
            }

            //ring pusher
            if (gamepad2.right_trigger > 0) {
                beyonce.RingPusherExtend();
            } else {
                beyonce.RingPusherRetract();
            }

            if (gamepad2.dpad_right) {
                beyonce.ShooterOn();
            }

            if (gamepad2.dpad_left) {
                beyonce.ShooterOff();
            }


            //keeps user updated
            telemetry.addData("Motors", "horizontal (%.2f), vertical (%.2f), pivot (%.2f)", horizontal, vertical, pivot);
            telemetry.update();



        }
    }


}
