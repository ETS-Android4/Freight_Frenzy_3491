package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robots.Beyonce;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;

@Autonomous
public class TestEncoder2 extends AutoOpMode {
    Beyonce beyonce = new Beyonce();
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override

    public void runOp() {


        beyonce.Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        beyonce.Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        addTimer();
        beyonce.Shooter.setPower(1);

        int velocity = 0;
        int tics1 = 0;
        int tics2 = 0;

        while (opModeIsActive()) {
            if (getSeconds() == 0) {
                tics1 = beyonce.Shooter.getPosition();
            }
            if(getSeconds() == 5) {
                tics2 = beyonce.Shooter.getPosition();
                velocity = (tics2 - tics1)/5;
                clearTimer();
            }

            telemetry.addData("Velocity: ", velocity);
            telemetry.update();
//            beyonce.Shooter.setPower(0.5);

        }
    }
}