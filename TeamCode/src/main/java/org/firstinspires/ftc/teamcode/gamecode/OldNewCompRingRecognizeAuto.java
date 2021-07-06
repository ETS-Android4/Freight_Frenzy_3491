/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.gamecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.Robots.Beyonce;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;

import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Disabled

@Autonomous
public class OldNewCompRingRecognizeAuto extends AutoOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private DcMotor shooter = null;
    private DcMotor arm = null;
    private DcMotor feeder;
    private DcMotor ziptiePuller;
    //public DcMotor Led;

    private CRServo beatInStick;
    private Servo claw;
    private Servo ramp = null;
    private Servo wallHolder;
    private Servo ringPusher;

    ColorSensor colorSensorL;
    ColorSensor colorSensorR;

     /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = RC.VUFORIA_LICENSE_KEY;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    @Override
    public void runOp() throws InterruptedException {

        frontLeft = hardwareMap.get(DcMotor.class, "frontL");
        frontRight = hardwareMap.get(DcMotor.class, "frontR");
        backLeft = hardwareMap.get(DcMotor.class, "backL");
        backRight = hardwareMap.get(DcMotor.class, "backR");

        shooter = hardwareMap.get(DcMotor.class, "Shooter");
        arm = hardwareMap.get(DcMotor.class, "Arm");
        ziptiePuller = hardwareMap.get(DcMotor.class, "ziptiepuller");
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        beatInStick = hardwareMap.get(CRServo.class, "BeatinStick");

        ramp = hardwareMap.get(Servo.class, "Ramp");
        wallHolder = hardwareMap.get(Servo.class, "WallHolder");
        ringPusher = hardwareMap.get (Servo.class, "RingPusher");
        claw = hardwareMap.get(Servo.class, "Claw");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Resetting Encoder");
        telemetry.update();

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        int state = 0;
        ColorSensor colorSensorL;
        colorSensorL = (ColorSensor) hardwareMap.get("ColourSensorL");

        ColorSensor colorSensorR;
        colorSensorR = (ColorSensor) hardwareMap.get("ColourSensorR");

        int redVal = 250;


        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();


            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(2.5, 1.78);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

//        shooterOn();
//        sleep(2000);
//        shoot();
//        shoot();
//        shoot();
//        shooterOff();
//        sleep(100);

        int objects;

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                int iterate = 0;
                boolean targetFound = false;
                //while (iterate < 10000) {
                if (tfod != null) {
                    //cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        objects = updatedRecognitions.size();
                        telemetry.addData("# Object Detected", objects);
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) { //arraylist
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());

                            if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                                state = 1;
                                telemetry.addData("state", 1);

                            } else if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                                state = 2;
                                telemetry.addData("state", 1);

                            }
                            telemetry.addData("in", 0);
                            break;
                        }

                        telemetry.addData("out", 0);
                        //telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        //recognition.getLeft(), recognition.getTop());
                        //telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        //recognition.getRight(), recognition.getBottom());

                        if (state == 0) {//if none
                            telemetry.addData("STATE", "0");
                            shooterOn();
                            sleep(2000);
                            shoot();
                            sleep(1000);
                            shoot();
                            sleep(1000);
                            shoot();
                            sleep(1000);
                            shooterOff();


                            clawClose();
                            driveBackwards(0.5);
                            sleep(1500);

                            while (opModeIsActive() && redVal > colorSensorL.red()){
                                strafeRight(0.5);
                                //red = opModeIsActive() && 120 < colorSensorL.red();
                                telemetry.addData("red", colorSensorR.red());
                            }
                            stopMotors();
                            telemetry.addData("Red Line 1", "Detected");

                            driveBackwards(0.5);
                            sleep(350);
                            stopMotors();
//
//                            strafeRight(0.5);
//                            sleep(200);
//                            stopMotors();

//                            while (opModeIsActive() && redVal > colorSensorL.red()){
//                                strafeRight(0.5);
//                                //red = opModeIsActive() && 120 < colorSensorL.red();
//                                telemetry.addData("red", colorSensorL.red());
//                            }
//                            stopMotors();
//                            telemetry.addData("Red Line 2", "Detected");

                            driveBackwards(0.5);
                            sleep(700);
                            stopMotors();

                            driveForward(0.5);
                            sleep(550);
                            stopMotors();
                            sleep(500);
                            armDown(-0.5);
                            sleep(1800);
                            armDown(-0.25);
                            sleep(250);
                            clawOpen();
                            sleep(200);
                            driveForward(0.5);
                            sleep(400);
                            stopMotors();

                            targetFound = true;
                        } else if (state == 1) { //if quad
                            telemetry.addData("quad:", "quad");

                            shooterOn();
                            sleep(2000);
                            shoot();
                            sleep(1000);
                            shoot();
                            sleep(1000);
                            shoot();
                            sleep(1000);
                            shooterOff();


                            clawClose();
                            strafeLeft(0.7);//i added this to straighten it out more
                            sleep(500);
                            driveBackwards(0.5);
                            sleep(1000);
                            strafeRight(0.5);
                            sleep(4000);

                            stopMotors();

                            //fowards until detect red
//                            while (opModeIsActive() && 120 > colorSensorL.red()){
//                                driveForward(0.3);
//                                //red = opModeIsActive() && 120 < colorSensorL.red();
//                                telemetry.addData("red", colorSensorL.red());
//                            }
//                            stopMotors();
//                            telemetry.addData("Red Line", "Detected");

                            driveBackwards(0.5);
                            sleep(500);
                            stopMotors();
                            sleep(500);
                            armDown(-0.5);
                            sleep(1800);
                            armDown(-0.25);
                            sleep(250);
                            sleep(500);
                            clawOpen();
                            sleep(500);
                            driveForward(0.5);
                            sleep(250);
                            strafeLeft(0.5);
                            sleep(1500);
                            stopMotors();
                            targetFound = true;
                        } else if (state == 2) { //if single
                            telemetry.addData("single", "single");

                            shooterOn();
                            sleep(2000);
                            shoot();
                            sleep(1000);
                            shoot();
                            sleep(1000);
                            shoot();
//                            sleep(1000);
                            shooterOff();
//

                            clawClose();
                            driveBackwards(0.2);
                            sleep(800);
                        ;

                            while (opModeIsActive() && redVal > colorSensorL.red()){
                                strafeRight(0.2);
                                //red = opModeIsActive() && 120 < colorSensorL.red();
                                telemetry.addData("red", colorSensorL.red());
                            }
                            stopMotors();
                            telemetry.addData("Red Line 1", "Detected");

                            driveBackwards(0.2);
                            sleep(200);
                            stopMotors();

                            strafeRight(0.2);
                            sleep(100);
                            stopMotors();

                            while (opModeIsActive() && redVal > colorSensorL.red()){
                                strafeRight(0.2);
                                //red = opModeIsActive() && 120 < colorSensorL.red();
                                telemetry.addData("red", colorSensorL.red());
                            }
                            stopMotors();
                            telemetry.addData("Red Line 2", "Detected");

//                            driveBackwards(0.7);
//                            sleep(500);
//                            stopMotors();

                            strafeRight(0.2);
                            sleep(300);
                            stopMotors();

                            while (opModeIsActive() && redVal > colorSensorL.red()){
                                strafeRight(0.2);
                                //red = opModeIsActive() && 120 < colorSensorL.red();
                                telemetry.addData("red", colorSensorL.red());
                            }
                            stopMotors();
                            telemetry.addData("Red Line 3", "Detected");

                            driveBackwards(0.2);
                            sleep(750);


                            while (opModeIsActive() && redVal > colorSensorR.red()){
                                driveForward(0.2);
                                //red = opModeIsActive() && 120 < colorSensorL.red();
                                telemetry.addData("red", colorSensorR.red());
                            }
                            telemetry.addData("line", "detect");
                            stopMotors();
                            sleep(500);

                            driveForward(0.2);
                            sleep(150);
                            stopMotors();

//                            while (opModeIsActive() && redVal > colorSensorR.red()){
//                                driveForward(0.7);
//                                //red = opModeIsActive() && 120 < colorSensorL.red();
//                                telemetry.addData("red", colorSensorR.red());
//                            }
//                            telemetry.addData("line", "detect");
//                            stopMotors();

                            sleep(500);
                            armDown(-0.2);
                            sleep(1800);
                            armDown(-0.25);
                            sleep(250);
                            sleep(500);
                            clawOpen();
                            sleep(500);
                            driveForward(0.2);
                            sleep(250);
                            strafeLeft(0.2);
                            sleep(700);
                            stopMotors();
                            targetFound = true;
                        } else {
                            shooterOn();
                            sleep(2000);
                            shoot();
                            sleep(1000);
                            shoot();
                            sleep(1000);
                            shoot();
                            sleep(1000);
                            shooterOff();


                            clawClose();
                            driveBackwards(0.2);
                            sleep(1500);

                            while (opModeIsActive() && redVal > colorSensorL.red()){
                                strafeRight(1);
                                //red = opModeIsActive() && 120 < colorSensorL.red();
                                telemetry.addData("red", colorSensorL.red());
                            }
                            stopMotors();
                            telemetry.addData("Red Line 1", "Detected");

                            driveBackwards(0.2);
                            sleep(200);
                            stopMotors();

                            strafeRight(0.2);
                            sleep(200);
                            stopMotors();

                            while (opModeIsActive() && redVal > colorSensorL.red()){
                                strafeRight(1);
                                //red = opModeIsActive() && 120 < colorSensorL.red();
                                telemetry.addData("red", colorSensorL.red());
                            }
                            stopMotors();
                            telemetry.addData("Red Line 2", "Detected");

                            driveBackwards(0.2);
                            sleep(700);
                            stopMotors();

                            driveForward(0.2);
                            sleep(1200);
                            stopMotors();
                            sleep(500);
                            armDown(-0.5);
                            sleep(1800);
                            armDown(-0.25);
                            sleep(250);
                            clawOpen();
                            sleep(200);
                            driveForward(0.2);
                            sleep(400);
                            stopMotors();
                            strafeLeft(0.2);
                            sleep(1000);
                            stopMotors();

                            targetFound = true;

                        }

                        telemetry.update();
                    }
                    if (targetFound) {
                        break;
                    }
                }
            }

            if (tfod != null) {
                tfod.shutdown();
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void driveForward(double speed){
        ((DcMotorEx)frontLeft).setVelocity(speed * 2000);
        ((DcMotorEx)frontRight).setVelocity(speed * 2000);
        ((DcMotorEx)backLeft).setVelocity(speed * 2000);
        ((DcMotorEx)backRight).setVelocity(speed * 2000);
    }

    private void driveBackwards(double speed){
        ((DcMotorEx)frontLeft).setVelocity(-speed * 2000);
        ((DcMotorEx)frontRight).setVelocity(-speed * 2000);
        ((DcMotorEx)backLeft).setVelocity(-speed * 2000);
        ((DcMotorEx)backRight).setVelocity(-speed * 2000);
    }

    private void strafeLeft(double speed){
        ((DcMotorEx)frontLeft).setVelocity(-speed * 2000);
        ((DcMotorEx)frontRight).setVelocity(speed * 2000);
        ((DcMotorEx)backLeft).setVelocity(speed * 2000);
        ((DcMotorEx)backRight).setVelocity(-speed * 2000);
    }

    private void strafeRight(double speed){
        ((DcMotorEx)frontLeft).setVelocity(speed * 2000);
        ((DcMotorEx)frontRight).setVelocity(-speed * 2000);
        ((DcMotorEx)backLeft).setVelocity(-speed * 2000);
        ((DcMotorEx)backRight).setVelocity(speed * 2000);
    }

    private void turnRight(double speed){
        ((DcMotorEx)frontLeft).setVelocity(speed * 2000);
        ((DcMotorEx)frontRight).setVelocity(-speed * 2000);
        ((DcMotorEx)backLeft).setVelocity(speed * 2000);
        ((DcMotorEx)backRight).setVelocity(-speed * 2000);
    }

    private void turnLeft(double speed){
        ((DcMotorEx)frontLeft).setVelocity(-speed * 2000);
        ((DcMotorEx)frontRight).setVelocity(speed * 2000);
        ((DcMotorEx)backLeft).setVelocity(-speed * 2000);
        ((DcMotorEx)backRight).setVelocity(speed * 2000);
    }

    private void stopMotors() {
        ((DcMotorEx)frontLeft).setVelocity(0);
        ((DcMotorEx)frontRight).setVelocity(0);
        ((DcMotorEx)backLeft).setVelocity(0);
        ((DcMotorEx)backRight).setVelocity(0);
    }

    private void moveArm(double joystickValue) {
        ((DcMotorEx)arm).setVelocity(joystickValue * 1440);
    }

    private void beat(double power) {
        beatInStick.setPower(power);
    }

    private void clawOpen() {
        claw.setPosition(0);
    }
    private void clawClose() {
        claw.setPosition(1);
    }

    private void shooterOn() {
        ((DcMotorEx)shooter).setVelocity(2800);
    }
    private void shooterOff() {
        ((DcMotorEx)shooter).setVelocity(0);
    }

    private double position = 0;
    private void moveRamp(boolean dPadUp, boolean dPadDown) {
        if (dPadUp && position < 0.8) {
            position = position + 0.001;
        } else if (dPadDown && position > 0.2) {
            position = position - 0.001;
        }
        ramp.setPosition(position);
    }

    public void armDown(double power) {
        ((DcMotorEx)arm).setVelocity(power * 1440);
    }

    private void armAuto(double power) {
        arm.setPower(power);
    }

    private void ringPusherExtend() {
        ringPusher.setPosition(1);
    }
    private void ringPusherRetract() {
        ringPusher.setPosition(0.45);
    }

    private void shoot() {
        ringPusherExtend();
        sleep(1250);
        ringPusherRetract();
        sleep(750);
        sleep(2500);
    }

    private void holdWall(){
        wallHolder.setPosition(0.2);
    }
    private void releaseWall(){
        wallHolder.setPosition(0.8);
    }
}
