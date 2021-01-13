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

    //Import Beyonce class
    Beyonce beyonce = new Beyonce();

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        //Indicate that the program is running
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Gives user 2 drive control options
        boolean leftHanded = true;

        //Standard code
        waitForStart();
        runtime.reset();

        //While opMode is active do the stuff in the while loop
        while (opModeIsActive()) {

            //Declaring drive variables
            double horizontal;
            double vertical;
            double pivot;

            //Drive control option one
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

            //Drive control option two
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

            //Drive calculations
            beyonce.FrontLeft.setPower(pivot + (vertical + horizontal));
            beyonce.FrontRight.setPower(-pivot + (vertical - horizontal));
            beyonce.BackLeft.setPower(pivot + (vertical - horizontal));
            beyonce.BackRight.setPower(-pivot + (vertical + horizontal));

            //Wobble grabber Arm
            beyonce.Arm.setPower(gamepad2.right_stick_y / 2);

            //Wobble grabber Claw
            if (gamepad2.x) {
                beyonce.ClawOpen();
            } else if (gamepad2.b){
                beyonce.ClawClose();
            }

            //Ring pusher
            if (gamepad2.right_trigger > 0) {
                beyonce.RingPusherExtend();
            } else {
                beyonce.RingPusherRetract();
            }

            //Shooter ramp
            beyonce.Ramp.setPower(gamepad2.left_stick_y);

            //Shooter
            if (gamepad2.dpad_right) {
                beyonce.ShooterOn();
            }
            if (gamepad2.dpad_left) {
                beyonce.ShooterOff();
            }

            //Keeps user updated
            telemetry.addData("Motors", "horizontal (%.2f), vertical (%.2f), pivot (%.2f)", horizontal, vertical, pivot);
            telemetry.update();
        }
    }
}