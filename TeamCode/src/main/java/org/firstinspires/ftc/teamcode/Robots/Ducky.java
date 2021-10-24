// Hardware and Autonomous set-up for Robot for 2021-2022 Freight Frenzy

package org.firstinspires.ftc.teamcode.Robots;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Ducky {

    // Declaring Motor Variables
    public DcMotor FrontLeft, BackLeft, FrontRight, BackRight;

    HardwareMap hwMap =  null;
    private ElapsedTime period  = new ElapsedTime();
    
    // Declaring stuff
    public Ducky(){

    }

    public void init(HardwareMap ahwMap)  {

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Drive Motors
        FrontLeft = hwMap.dcMotor.get("frontL");
        BackLeft = hwMap.dcMotor.get("backL");
        FrontRight = hwMap.dcMotor.get("frontR");
        BackRight = hwMap.dcMotor.get("backR");

        // Setting Motor Direction
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);

        // Motor set Power
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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