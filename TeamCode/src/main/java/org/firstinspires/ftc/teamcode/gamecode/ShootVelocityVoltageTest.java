package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.newhardware.Motor;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.opmodesupport.TeleOpMode;

@Disabled

@Autonomous
public class ShootVelocityVoltageTest extends AutoOpMode {

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private DcMotor shooter = null;
    private DcMotor arm = null;
    private Motor feeder;
    private Motor ziptiePuller;
    //public DcMotor Led;

    private CRServo beatInStick;
    private Servo claw;
    private Servo ramp;
    private Servo wallHolder;
    private Servo ringPusher;

    private VoltageSensor voltageSensor;


    @Override
    public void runOp() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "frontL");
        frontRight = hardwareMap.get(DcMotor.class, "frontR");
        backLeft = hardwareMap.get(DcMotor.class, "backL");
        backRight = hardwareMap.get(DcMotor.class, "backR");

        shooter = hardwareMap.get(DcMotor.class, "Shooter");
        arm = hardwareMap.get(DcMotor.class, "Arm");
        ziptiePuller = new Motor("ziptiepuller");
        feeder = new Motor("feeder");
        beatInStick = hardwareMap.get(CRServo.class, "BeatinStick");

        ramp = hardwareMap.get(Servo.class, "Ramp");
        wallHolder = hardwareMap.get(Servo.class, "WallHolder");
        ringPusher = hardwareMap.get (Servo.class, "RingPusher");
        claw = hardwareMap.get(Servo.class, "Claw");

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Expansion Hub 2");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        double startRampPosition = 0;

        if(voltageSensor.getVoltage() > 13) {
            startRampPosition = 0.45;
        } else if (voltageSensor.getVoltage() < 13 && voltageSensor.getVoltage() >= 12.9) {
            startRampPosition = 0.42;
        } else if (voltageSensor.getVoltage() < 12.9 && voltageSensor.getVoltage() >= 12.7) {
            startRampPosition = 0.42;
        } else if (voltageSensor.getVoltage() < 12.7 && voltageSensor.getVoltage() >= 12.5) {
            startRampPosition = 0.42;
        } else if (voltageSensor.getVoltage() < 12.5 && voltageSensor.getVoltage() >= 12.3) {
            startRampPosition = 0.42;
        } else if (voltageSensor.getVoltage() < 12.3 && voltageSensor.getVoltage() >= 12.1) {
            startRampPosition = 0.42;
        }

        ramp.setPosition(startRampPosition);
        sleep(2000);

        shoot();
        sleep(1000);

        shoot();
        sleep(1000);

        shoot();
        sleep(1000);
    }

    private void shoot() {
        ringPusherExtend();
        sleep(200);
        ringPusherRetract();
    }

    private void ringPusherExtend() {
        ringPusher.setPosition(1);
    }

    private void ringPusherRetract() {
        ringPusher.setPosition(0.45);
    }

}
