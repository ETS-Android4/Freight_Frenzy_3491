package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode" using a shooter motor with encoder.
 * It makes use of the RUN_USING_ENCODER mode of the DcMotor class.
 *
 * However, you need to use the DcMotorEx class to really work with encoders.
 * The basic DcMotor class only really allows Run to Position.
 *
 * With DCMotorEx you can set a velocity in ticks per second.
 * You can also get the current velocity, even modify the PID constants.
 *
 * Basically, you use setVelocity instead of setPower. See code below.
 * It should maintain that velocity even as the batter power changes,
 * as long as your not trying to maintain 100% motor power.
 *
 * Probably worth checking the Javadoc for DCMotorEx and/or look for other examples.
 * But setVelocity might be all you need for a shooter.
 *
 * The javadoc mentioned how one should use a Java 'cast' to use DcMotorEx
 * e.g. ((DcMotorEx) shooter) will 'cast' a DcMotor object into a DcMotorEx object.
 * That way we can reference the setVelocity method as follows:
 * ((DcMotorEx) shooter).setVelocity(x);
 *
 */
@Disabled

@Autonomous(name="Encoder2", group="Linear Opmode")

public class Encoder2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor shooter = null;
    double shooterPower = 0.0;  // not used in this example
    double shooterVelocity = 0.0;  // used to read velocity in this example

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        shooter = hardwareMap.get(DcMotor.class, "Shooter");

        telemetry.addData("Status", "Resetting Encoder");
        telemetry.update();

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ((DcMotorEx) shooter).setVelocity(0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        ((DcMotorEx) shooter).setVelocity(500); // start motor moving
        // for Ultraplanetary motor values can be from 0 to just over 2000
        // you can setPower to 1.0 and use telemetry to read getVelocity to see max velocity.
        // I noticed in telemetry that the getVelocity values would ramp up to the target fairly quickly,
        // then bounce around a little, but stay close to the target that was set.

        while (opModeIsActive()) {
            // run until the end of the match (driver presses STOP). All we're doing here is telemetry.

            shooterVelocity = ((DcMotorEx) shooter).getVelocity();

            // Show the elapsed game time and shooter velocity.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Shooter", "velocity: %.0f", shooterVelocity);
            telemetry.update();
        }
        // STOP was pressed
        ((DcMotorEx) shooter).setVelocity(0);
    }
}
