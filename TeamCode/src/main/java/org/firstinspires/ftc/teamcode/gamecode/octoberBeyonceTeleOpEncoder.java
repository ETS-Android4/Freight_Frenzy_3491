package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robots.Beyonce;
import org.firstinspires.ftc.teamcode.Robots.BeyonceEncoderTest;


@TeleOp
public class octoberBeyonceTeleOpEncoder extends LinearOpMode {
    BeyonceEncoderTest beyonce = new BeyonceEncoderTest(hardwareMap);

    //probably going to be useful. if it aint broke, dont fix it
    private ElapsedTime runtime = new ElapsedTime();

    //motors
    private DcMotorEx frontLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;




    public void runOpMode() {
        //indicate that the program is running
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //find moters in the configuration file on phones
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontL");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontR");
        backLeft = hardwareMap.get(DcMotor.class, "backL");
        backRight = hardwareMap.get(DcMotor.class, "backR");


        //set all motors to drive in the same direction in the real world
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//nice  //gives user 2 drive control options
        boolean weirdDriveControlsThatWasUsedLastYearThatIshaanCannotStandAndAbsolutlyHates = true;

        //creates 3 vertical aiming options
        int target = 1; //for the target ramp

        boolean shooterToggle = false;
        int toggleTimer = 0;

        waitForStart();
        runtime.reset();


        //while opMode is active do the stuff in the while loop
        while (opModeIsActive()) {



            //so that i dont have to write gamepad1.left_stick every single time for the calculations
            double horizontal;
            double vertical;
            double pivot;


            if (gamepad1.right_trigger > 0) {
                pivot = gamepad1.right_stick_x / 3;
                horizontal = -gamepad1.left_stick_x / 3;
                vertical = gamepad1.left_stick_y / 3;
            } else {
                pivot = gamepad1.right_stick_x;
                horizontal = -gamepad1.left_stick_x;
                vertical = gamepad1.left_stick_y;
            }


            //drive calculations
            frontLeft.setPower(pivot + (vertical + horizontal));
            frontRight.setPower(-pivot + (vertical - horizontal));
            backLeft.setPower(pivot + (vertical - horizontal));
            backRight.setPower(-pivot + (vertical + horizontal));

            double x = frontLeft.getCurrentPosition();
            double y = frontRight.getCurrentPosition();


            //keeps user updated
            telemetry.addData("Moved", "x = (%.2f), x = (%.2f)", x, y);
            telemetry.update();


        }
    }
}
