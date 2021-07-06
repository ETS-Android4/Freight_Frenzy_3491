package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robots.Beyonce;
import org.firstinspires.ftc.teamcode.opmodesupport.TeleOpMode;

@Disabled

@TeleOp
public class NovemberBeyonceteleop extends TeleOpMode {
    Beyonce beyonce;
    //motors
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private boolean ShooterOn = false;
    private boolean buttonpressed = false;

    @Override
    public void initialize() {
        beyonce = new Beyonce();
        //beyonce.init();
        //motors

        frontLeft = hardwareMap.get(DcMotor.class, "frontL");
        frontRight = hardwareMap.get(DcMotor.class, "frontR");
        backLeft = hardwareMap.get(DcMotor.class, "backL");
        backRight = hardwareMap.get(DcMotor.class, "backR");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    //gamepad shortcut booleans
    boolean leftHandControls = true;
    double horizontal;
    double vertical;
    double pivot;

    @Override
    public void loopOpMode() {

        if (leftHandControls) {
            if (gamepad1.left_bumper) {
                pivot = gamepad1.left_stick_x / 3;
                horizontal = -gamepad1.right_stick_x / 3;
                vertical = gamepad1.right_stick_y / 3;
            } else {
                pivot = gamepad1.left_stick_x;
                horizontal = -gamepad1.right_stick_x;
                vertical = gamepad1.right_stick_y;
            }

            if ((gamepad1.dpad_down) && (gamepad1.a)) {
                leftHandControls = false;
            }
        } else {
            if (gamepad1.right_trigger > 0) {
                pivot = gamepad1.right_stick_x / 3;
                horizontal = -gamepad1.left_stick_x / 3;
                vertical = gamepad1.left_stick_y / 3;
//nice
                telemetry.addData("right trigger value: ", gamepad1.right_trigger);
            } else {
                pivot = gamepad1.right_stick_x;
                horizontal = -gamepad1.left_stick_x;
                vertical = gamepad1.left_stick_y;
            }

            if (gamepad1.dpad_down && gamepad1.b) {
                leftHandControls = true;
            }
        }


        frontLeft.setPower(pivot + (vertical + horizontal));
        frontRight.setPower(-pivot + (vertical - horizontal));
        backLeft.setPower(pivot + (vertical - horizontal));
        backRight.setPower(-pivot + (vertical + horizontal));


//        //Linear Slide
//        if (gamepad2.a){
//            beyonce.LinearSlidesUp();
//        } else if (gamepad2.y){
//            beyonce.LinearSlidesDown();
//        } else {
//            beyonce.LinearSidesStop();
//        }

        //Wobble Grabber
//        if (gamepad2.b) {
//            beyonce.GrabberUp();
//        } else if (gamepad2.x){
//            beyonce.GrabberDown();
//        }

        //ring pusher
        if (gamepad2.right_trigger > 0) {
            beyonce.RingPusherExtend();
        } else {
            beyonce.RingPusherRetract();
        }

        if (gamepad2.dpad_right){ //if the button is pressed
            buttonpressed = true;
        }


        if (!(gamepad2.dpad_right)){ //if the button IS not pressed
            if (buttonpressed) { //if the button WAS pressed

                if (ShooterOn) {
                    ShooterOn = false;
                }

                else if (ShooterOn == false) {
                    ShooterOn = true;
                }

            }

            buttonpressed = false;
        }

        if (ShooterOn){
            beyonce.ShooterOn();
            telemetry.addData("shooter", "on");
        }
        if (!(ShooterOn)){
            beyonce.ShooterOff();
            telemetry.addData("shooter", "off");
        }




    }
}