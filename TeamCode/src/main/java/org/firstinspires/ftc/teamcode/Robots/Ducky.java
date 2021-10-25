// Hardware and Autonomous set-up for Robot for 2021-2022 Freight Frenzy

package org.firstinspires.ftc.teamcode.Robots;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Ducky {

    // Declaring Motor Variables
    public DcMotor FrontLeft, BackLeft, FrontRight, BackRight;

    // Declaring opMode Variables
    HardwareMap hwMap;

    // Encoder + Wheel Declaration
    public static final double MOTOR_TICK_COUNTS_PER_REV = 0; // Encoder counts per Motor (5203-2402-0019) Revolution

    public static final double OMNI_WHEEL_DIAMETER =  96; // Diameter of Omni Wheel in mm
    public static final double CART_WHEEL_DIAMETER = 127; // Diameter of Cart Wheel in mm


    // Class Constructor
    public Ducky(){

    }

    public void init(HardwareMap ahwMap)  {

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define Drive Motors
        FrontLeft = hwMap.dcMotor.get("frontL");
        BackLeft = hwMap.dcMotor.get("backL");
        FrontRight = hwMap.dcMotor.get("frontR");
        BackRight = hwMap.dcMotor.get("backR");

        // Setting Motor Direction
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);

        // Define Servos
//        leftClaw  = hwMap.get(Servo.class, "left_hand");

        // Initialize Servos
//        leftClaw.setPosition(MID_SERVO);

        // Motor set Power at init
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);

        // Setting Motors to run with/ without Encoders - (RUN_WITHOUT_ENCODER/ RUN_USING_ENCODER)
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Autonomous
     */
    // Robot Driving
    public void DriveForward(double speed, int milliseconds){
        FrontLeft.setPower(speed);
        BackLeft.setPower(speed/ 0.7559);
        FrontRight.setPower(speed);
        BackRight.setPower(speed/ 0.7559);
        sleep(milliseconds);
        Stop();
    }
    public void DriveBackward(double speed, int milliseconds){
        FrontLeft.setPower(-speed);
        BackLeft.setPower(-speed/ 0.7559);
        FrontRight.setPower(-speed);
        BackRight.setPower(-speed/ 0.7559);
        sleep(milliseconds);
        Stop();
    }
    public void TurnLeft(double speed, int milliseconds){
        FrontLeft.setPower(-speed);
        BackLeft.setPower(-speed/ 0.7559);
        FrontRight.setPower(speed);
        BackRight.setPower(speed/ 0.7559);
        sleep(milliseconds);
        Stop();
    }
    public void TurnRight(double speed, int milliseconds){
        FrontLeft.setPower(speed);
        BackLeft.setPower(speed/ 0.7559);
        FrontRight.setPower(-speed);
        BackRight.setPower(-speed/ 0.7559);
        sleep(milliseconds);
        Stop();
    }
    public void Stop(){
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);
    }
}