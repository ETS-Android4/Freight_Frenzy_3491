//package org.firstinspires.ftc.teamcode.gamecode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Robots.Beyonce;
//
//@Disabled
//
//@TeleOp
//public class BasicTeleOp extends LinearOpMode {
//    Beyonce beyonce = new Beyonce();
//
//
//    private ElapsedTime runtime = new ElapsedTime();
//
//    public void runOpMode() {
//        //Telemetry Data
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        //Predetermined Target Ramp positions
//        int target = 0;
//
//        //Shooter Running
////        int ShooterRunning = 1;
//
//        waitForStart();
//        runtime.reset();
//
//        //while opMode is active do the stuff in the while loop
//        while (opModeIsActive()) {
//
//            ////Driving Controls
//            //Slow Button
//            if (gamepad1.left_trigger < 0) {
//                float pivot = gamepad1.right_stick_y / 2;
//                float horizontal = -gamepad1.right_stick_x / 2;
//                float vertical = gamepad1.left_stick_x / 2;
//                Beyonce.FrontRight.setPower(-pivot + (vertical + horizontal));
//                Beyonce.BackRight.setPower(-pivot + (vertical - horizontal));
//                Beyonce.FrontLeft.setPower(pivot + (vertical + horizontal));
//                Beyonce.BackLeft.setPower(pivot + (vertical - horizontal));
//
//                //Normal Driving
//            } else {
//                float pivot = gamepad1.right_stick_y;
//                float horizontal = -gamepad1.right_stick_x;
//                float vertical = gamepad1.left_stick_x;
//                Beyonce.FrontRight.setPower(-pivot + (vertical + horizontal));
//                Beyonce.BackRight.setPower(-pivot + (vertical - horizontal));
//                Beyonce.FrontLeft.setPower(pivot + (vertical + horizontal));
//                Beyonce.BackLeft.setPower(pivot + (vertical - horizontal));
//            }
//
//            //Linear Slide
//            if (gamepad2.y){
//                Beyonce.LinearSlide.setPower(1);
//            } else if (gamepad2.b){
//                Beyonce.LinearSlide.setPower(-1);
//            } else {
//                Beyonce.LinearSlide.setPower(0);
//            }
//
//            //Wobble Grabber
//            if (gamepad2.a) {
//                Beyonce.Grabber.setPosition(1);
//            } else if (gamepad2.x){
//                Beyonce.Grabber.setPosition(0);
//            }
//
//            ////Shooter
//            if(gamepad2.left_trigger < 0) {
//                Beyonce.Shooter.setPower(1);
//                telemetry.addData("Shooter:", "Powering On...");
//                telemetry.update();
//
//                //Wait until Shooter is at full speed
//                sleep(5000);
//                telemetry.addData("Shooter:", "Full Speed...");
//                telemetry.update();
//            } else {
//                Beyonce.Shooter.setPower(0);
//                telemetry.addData("Shooter:", "Off.");
//                telemetry.update();
//            }
//
////            Shooter Trigger -
////            if (ShooterRunning == 0) {
////                Beyonce2.Shooter.setPower(0);
////                telemetry.addData("Shooter:", "Powering On...");
////                telemetry.update();
////
////                //Wait until Shooter is at full speed
////                sleep(5000);
////                telemetry.addData("Shooter:", "Full Speed...");
////                telemetry.update();
////            } else {
////                Beyonce2.Shooter.setPower(1);
////            }
//
//            ////Ring Pusher
//            if (gamepad2.right_trigger > 0){
//                Beyonce.RingPusher.setPosition(1);
//                sleep(20);
//                Beyonce.RingPusher.setPosition(0);
//            }
//
////            //Target Ramp
////            if (gamepad2.dpad_up == true) {
////                //
////                if (target == 0) {
////                    Beyonce2.TargetRamp.setPosition(0);
////                } else if (target == 1) {
////                    Beyonce2.TargetRamp.setPosition(0.2);
////                } else if (target == 2) {
////                    Beyonce2.TargetRamp.setPosition(0.4);
////                } else if (target == 3) {
////                    Beyonce2.TargetRamp.setPosition(1);
////                }
////            }
//        }
//    }
//}