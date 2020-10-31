package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class shooterOn extends LinearOpMode {

    //probably going to be useful. if it aint broke, dont fix it
    private ElapsedTime runtime = new ElapsedTime();

    //motors
    private DcMotor shooter = null;

    //servos
    //private Servo targetRamp = null;
    private Servo ringPusher = null;


    public void runOpMode() {
        //indicate that the program is running
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //find moters in the configuration file on phones
        shooter = hardwareMap.get(DcMotor.class, "Shooter");
        ringPusher = hardwareMap.get(Servo.class, "RingPusher");



        waitForStart();
        runtime.reset();

        //while opMode is active do the stuff in the while loop
        while (opModeIsActive()) {
            shooter.setPower(1);

            if (gamepad2.right_trigger > 0) {
                ringPusher.setPosition(0.2);
            } else {
                ringPusher.setPosition(0.8);
            }
        }
    }
}
