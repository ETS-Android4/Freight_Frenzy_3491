package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robots.Beyonce2;


@TeleOp(name="BasicTeleOp", group="Linear Opmode")
public class BasicTeleOp extends LinearOpMode {
    //Beyonce2 Beyonce = new Beyonce2();
    Beyonce2 Beyonce2;

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        //Telemetry Data
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //gives user 2 drive control options
        boolean weirdDriveControlsThatWasUsedLastYearThatIshaanCannotStandAndAbsolutlyHates = true;

        //creates 3 vertical aiming options
        int target = 1;

        waitForStart();
        runtime.reset();

        //while opMode is active do the stuff in the while loop
        while (opModeIsActive()) {

        }
    }
}



/*package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodesupport.TeleOpMode;
import org.firstinspires.ftc.teamcode.robots.HansonNightmare;

@TeleOp
public class BasicTeleOpHanson extends TeleOpMode {
    HansonNightmare HansonNightmare;

    @Override
    public void initialize() {
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loopOpMode() {
        if (joy1.leftTrigger()){
            telemetry.addData("Status", "Reverse");
            HansonNightmare.driveR(gamepad1.left_stick_y*0.9);
            HansonNightmare.driveL(gamepad1.right_stick_y*0.9);
        }

        if (!joy1.leftTrigger()) {
            telemetry.addData("Status", "Normal Driving");
            HansonNightmare.driveL(-gamepad1.left_stick_y);
            HansonNightmare.driveR(-gamepad1.right_stick_y);
        }
    }
}*/