//package org.firstinspires.ftc.teamcode.z_OldPrograms;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.opMode_Support.TeleOpMode;
//
//@TeleOp
//public class SaanichFair extends TeleOpMode {
//
//
//    private DcMotor shooter = null;
//    private Servo ramp;
//    private Servo ringPusher;
//
//    @Override
//    public void initialize() {
//
//        shooter = hardwareMap.get(DcMotor.class, "Shooter");
//        ramp = hardwareMap.get(Servo.class, "Ramp");
//        ringPusher = hardwareMap.get (Servo.class, "RingPusher");
//
//    }
//
//    @Override
//    public void loopOpMode() {
//        if (gamepad1.dpad_right) {
//            shooterOn();
//        }
//        if (gamepad1.dpad_left) {
//            shooterOff();
//        }
//        moveRamp(gamepad1.left_stick_y, gamepad1.dpad_down);
//
//        if (gamepad1.right_trigger > 0) {
//            ringPusherExtend();
//        } else {
//            ringPusherRetract();
//        }
//
//    }
//    private double position = 0.6;
//    private void moveRamp(double left_stick_up, boolean dPadDown) {
//        if (left_stick && position < 0.8) {
//            position = position + 0.001;
//        } else if (left_stick && position > 0.2) {
//            position = position - 0.001;
//        }
//        ramp.setPosition(position);
//    }
//    private void shooterOn() {
//        ((DcMotorEx)shooter).setVelocity(2800);
//    }
//    private void shooterOff() {
//        ((DcMotorEx)shooter).setVelocity(0);
//    }
//
//    private void ringPusherExtend() {
//        ringPusher.setPosition(1);
//    }
//    private void ringPusherRetract() {
//        ringPusher.setPosition(0.45);
//    }
//
//}
