package org.firstinspires.ftc.teamcode.Competition.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Season_Setup.Ducky;


@Autonomous(name="Ramming through Barrier - P1, Blue")

public class RammingThroughBarrier_P1_Blue extends LinearOpMode {

    // Initializing Robot Class
    Ducky ducky = new Ducky();

    @Override
    public void runOpMode() throws InterruptedException{

        // Initialize all motors/ servos
        ducky.init(hardwareMap, telemetry);

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Waiting for the program to start
        waitForStart();

        // Autonomous Pathing
        DriveBackward_Power(0.2,1000);
    }

    // Robot Driving (Power and Time only)
    public void DriveForward_Power(double speed, int milliseconds) throws InterruptedException {
        ducky.FrontLeft.setPower(speed);
        ducky.BackLeft.setPower(speed / Ducky.BACK_WHEEL_POWER_REDUCTION);
        ducky.FrontRight.setPower(speed);
        ducky.BackRight.setPower(speed / Ducky.BACK_WHEEL_POWER_REDUCTION);
        Thread.sleep(milliseconds);
        Stop_Power();
    }
    public void DriveBackward_Power(double speed, int milliseconds) throws InterruptedException {
        ducky.FrontLeft.setPower(-speed);
        ducky.BackLeft.setPower(-speed / Ducky.BACK_WHEEL_POWER_REDUCTION);
        ducky.FrontRight.setPower(-speed);
        ducky.BackRight.setPower(-speed / Ducky.BACK_WHEEL_POWER_REDUCTION);
        Thread.sleep(milliseconds);
        Stop_Power();
    }
    public void TurnLeft_Power(double speed, int milliseconds) throws InterruptedException {
        ducky.FrontLeft.setPower(-speed);
        ducky.BackLeft.setPower(-speed / Ducky.BACK_WHEEL_POWER_REDUCTION);
        ducky.FrontRight.setPower(speed);
        ducky.BackRight.setPower(speed / Ducky.BACK_WHEEL_POWER_REDUCTION);
        Thread.sleep(milliseconds);
        Stop_Power();
    }
    public void TurnRight_Power(double speed, int milliseconds) throws InterruptedException {
        ducky.FrontLeft.setPower(speed);
        ducky.BackLeft.setPower(speed / Ducky.BACK_WHEEL_POWER_REDUCTION);
        ducky.FrontRight.setPower(-speed);
        ducky.BackRight.setPower(-speed / Ducky.BACK_WHEEL_POWER_REDUCTION);
        Thread.sleep(milliseconds);
        Stop_Power();
    }
    public void Stop_Power() {
        ducky.FrontLeft.setPower(0);
        ducky.BackLeft.setPower(0);
        ducky.FrontRight.setPower(0);
        ducky.BackRight.setPower(0);
    }
}
