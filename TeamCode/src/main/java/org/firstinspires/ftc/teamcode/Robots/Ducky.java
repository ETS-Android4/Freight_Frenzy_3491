// Hardware and Autonomous set-up for Robot for 2021-2022 Freight Frenzy

package org.firstinspires.ftc.teamcode.Robots;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Ducky {

    // Declaring Motor Variables
    public DcMotor FrontLeft, BackLeft, FrontRight, BackRight;

    // Declaring stuff
    public Ducky(HardwareMap hardwareMap) throws InterruptedException {

        // Drive Motors
        FrontLeft = hardwareMap.dcMotor.get("frontL");
        BackLeft = hardwareMap.dcMotor.get("backL");
        FrontRight = hardwareMap.dcMotor.get("frontR");
        BackRight = hardwareMap.dcMotor.get("backR");

        // Setting Motor Direction
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
    }

    /**
     * Autonomous
     */
    // Robot Driving
    public void DriveForward(double speed){
        FrontLeft.setPower(speed);
        BackLeft.setPower(speed);
        FrontRight.setPower(speed);
        BackRight.setPower(speed);
    }
    public void DriveBackward(double speed){
        FrontLeft.setPower(-speed);
        BackLeft.setPower(-speed);
        FrontRight.setPower(-speed);
        BackRight.setPower(-speed);
    }
    public void TurnLeft(double speed){
        FrontLeft.setPower(-speed);
        BackLeft.setPower(-speed);
        FrontRight.setPower(speed);
        BackRight.setPower(speed);
    }
    public void TurnRight(double speed){
        FrontLeft.setPower(speed);
        BackLeft.setPower(speed);
        FrontRight.setPower(-speed);
        BackRight.setPower(-speed);
    }
    public void Stop(){
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);
    }
}