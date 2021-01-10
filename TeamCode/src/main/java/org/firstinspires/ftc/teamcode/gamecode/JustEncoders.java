package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
//import org.firstinspires.ftc.teamcode.Robots.Beyonce;

@Autonomous
public class JustEncoders extends AutoOpMode {
    DcMotorEx frontRight;
    DcMotorEx frontLeft;
    DcMotorEx backRight;
    DcMotorEx backLeft;

    double pivot;
    double horizontal;
    double vertical;

    //Beyonce beyonce = new Beyonce();

    @Override
    public void runOp() {
        frontRight = hardwareMap.get(DcMotorEx.class, "frontR");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontL");
        backRight = hardwareMap.get(DcMotorEx.class, "backR");
        backLeft = hardwareMap.get(DcMotorEx.class, "backL");

        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();


        telemetry.addData("Encoder y value", backRight.getCurrentPosition());
        telemetry.addData("Encoder x value", backLeft.getCurrentPosition());
        telemetry.update();

        DriveForwardsEncoder(1, 10000);
        StrafeRightEncoder(1, 5000);

    }

    public void DriveForwardsEncoder(double speed, int ticks){
        while (backRight.getCurrentPosition() < ticks) {
            frontLeft.setPower(speed);
            frontRight.setPower(speed);
            backLeft.setPower(speed);
            backRight.setPower(speed);
            telemetry.addData("Encoder y value", backRight.getCurrentPosition());
            telemetry.addData("Encoder x value", backLeft.getCurrentPosition());
            telemetry.update();
            Stop();
        }
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void StrafeRightEncoder(double speed, int ticks){
        while (backLeft.getCurrentPosition() < ticks) {
            frontLeft.setPower(speed);
            frontRight.setPower(-speed);
            backLeft.setPower(-speed);
            backRight.setPower(speed);
            telemetry.addData("Encoder y value", backRight.getCurrentPosition());
            telemetry.addData("Encoder x value", backLeft.getCurrentPosition());
            telemetry.update();
            Stop();
        }
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    public void Stop(){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}





























//nice