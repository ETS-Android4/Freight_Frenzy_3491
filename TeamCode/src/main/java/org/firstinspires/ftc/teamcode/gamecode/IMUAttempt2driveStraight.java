package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled

@Autonomous(name="IMUAttempt2", group="Linear Opmode")
public class IMUAttempt2driveStraight extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        backLeftDrive = hardwareMap.get(DcMotor.class, "backL");
        backRightDrive = hardwareMap.get(DcMotor.class, "backR");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontR");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Setup a variable for each drive wheel to save power level for telemetry
        double pwrlvl = 0.5;

        //drives forward for 1 second, then stops and displays an updat



        telemetry.addData("Status", "Initialized");

        while (opModeIsActive()) {
            DriveForward();
        }

        DriveReset();

    }


    //methods

    private void DriveReset() {
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
    }
//nice
    public void DriveForward() {
        backLeftDrive.setPower(0.75);
        backRightDrive.setPower(0.75);
        frontLeftDrive.setPower(0.75);
        frontRightDrive.setPower(0.75);
    }

    public void DriveBackward(double pwrlvl){
        backLeftDrive.setPower(-pwrlvl);
        backRightDrive.setPower(-pwrlvl);
    }

    public void TurnLeft(double pwrlvl){
        backLeftDrive.setPower((pwrlvl - 0.75)/2);
        backRightDrive.setPower(pwrlvl/2);
        frontLeftDrive.setPower((pwrlvl - 0.75)/2);
        frontRightDrive.setPower(pwrlvl/2);
    }

    public void TurnRight(double pwrlvl){

        BNO055IMU imu;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        float turnValue = Math.abs(angles.firstAngle);
        backLeftDrive.setPower(pwrlvl/2);
        backRightDrive.setPower(pwrlvl/-2);
        frontLeftDrive.setPower(pwrlvl/2);
        frontRightDrive.setPower(pwrlvl/-2);

        while (turnValue < 90) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            turnValue = Math.abs(angles.firstAngle);
            sleep(200); //Turn until you almost reach 90deg

            telemetry.addData("angle1", angles.firstAngle);
            telemetry.update();

        }
    }
}
