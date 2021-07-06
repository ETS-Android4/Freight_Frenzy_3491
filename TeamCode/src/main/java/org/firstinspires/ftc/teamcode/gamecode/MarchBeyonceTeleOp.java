package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Beyonce;
import org.firstinspires.ftc.teamcode.opmodesupport.TeleOpMode;

@Disabled

@TeleOp
public class MarchBeyonceTeleOp extends TeleOpMode {
    Beyonce beyonce;

    public void initialize() {
        //Create object of beyonce
        beyonce = new Beyonce();
        beyonce.FrontLeft.setReverse(true);
        beyonce.BackLeft.setReverse(true);

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
            beyonce.FrontLeft.setPower((pivot + (vertical + horizontal))/5);
            beyonce.FrontRight.setPower((-pivot + (vertical - horizontal))/5);
            beyonce.BackLeft.setPower((pivot + (vertical - horizontal))/5);
            beyonce.BackRight.setPower((-pivot + (vertical + horizontal))/5);
            telemetry.addData("s", "slow");
        } else {
            beyonce.FrontLeft.setPower(pivot + (vertical + horizontal));
            beyonce.FrontRight.setPower(-pivot + (vertical - horizontal));
            beyonce.BackLeft.setPower(pivot + (vertical - horizontal));
            beyonce.BackRight.setPower(-pivot + (vertical + horizontal));
            telemetry.addData("s", "fast");
        }
//nice

        //Wobble grabber Arm
        beyonce.Arm.setPower(gamepad2.right_stick_y / 4);

        //Wobble grabber Claw
        if (gamepad2.b) {
            beyonce.ClawClose();
            telemetry.addData("Claw ", "closing");
        } else if (gamepad2.x) {
            beyonce.ClawOpen();
            telemetry.addData("claw", "open");
        }

        beyonce.Beat(gamepad2.left_stick_x);


        if (gamepad2.left_trigger > 0){
            beyonce.HoldWall();
            telemetry.addData("wall", "hold");
        }
        else if (gamepad2.left_bumper){
            beyonce.ReleaseWall();
            telemetry.addData("wall", "release");
        }



//        if (gamepad2.y) {
//            beyonce.Led.setPower(0);
//        }
//        beyonce.Led.setPower(1);

        //Ring pusher
        if (gamepad2.right_trigger > 0) {
            beyonce.RingPusherExtend();
        } else {
            beyonce.RingPusherRetract();
        }

        //Shooter ramp
        //beyonce.moveRamp(gamepad1.left_stick_y);

        //rightStick

        //Shooter
        if (gamepad2.dpad_right) {
            beyonce.ShooterOn();
        }
        if (gamepad2.dpad_left) {
            beyonce.ShooterOff();
        }

        if (gamepad1.left_bumper){
            beyonce.Ramp.setPosition(0.8);
        }
        if (gamepad1.right_bumper){
            beyonce.Ramp.setPosition(0.2);
        }


        if (gamepad2.a){
            beyonce.feeder.setPower(-1);

            beyonce.ziptiepuller.setPower(0.5);
        }

        if (gamepad2.y){
            beyonce.feeder.setPower(0);

            beyonce.ziptiepuller.setPower(0);
        }

        if (gamepad2.left_stick_button) {
            beyonce.feeder.setPower(1);

            beyonce.ziptiepuller.setPower(-0.5);
        }



        //Keeps user updated
        telemetry.addData("arm:", (gamepad2.right_stick_y/4));
        telemetry.update();

    }
}