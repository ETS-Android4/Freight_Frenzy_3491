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
import com.qualcomm.robotcore.hardware.ColorSensor;

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
@Autonomous
public class CompRingRecognizeAutoWITHSTRAFE extends AutoOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    Beyonce beyonce = new Beyonce();
     /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
nice * Vuforia license keys are always 380 characters long, and look as if they contain mostly
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

//        beyonce.ShooterOn();
//        sleep(2000);
//        beyonce.Shoot();
//        beyonce.Shoot();
//        beyonce.Shoot();
//        beyonce.ShooterOff();
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
                            beyonce.ShooterOn();
                            sleep(2000);
                            beyonce.Shoot();
                            sleep(1000);
                            beyonce.Shoot();
                            sleep(1000);
                            beyonce.Shoot();
                            sleep(1000);
                            beyonce.ShooterOff();


                            beyonce.ClawClose();
                            beyonce.DriveBackward(0.5);
                            sleep(1500);

                            while (opModeIsActive() && redVal > colorSensorL.red()){
                                beyonce.StrafeRight(0.5);
                                //red = opModeIsActive() && 120 < colorSensorL.red();
                                telemetry.addData("red", colorSensorR.red());
                            }
                            beyonce.Stop();
                            telemetry.addData("Red Line 1", "Detected");

                            beyonce.DriveBackward(0.5);
                            sleep(350);
                            beyonce.Stop();
//
//                            beyonce.StrafeRight(0.5);
//                            sleep(200);
//                            beyonce.Stop();

//                            while (opModeIsActive() && redVal > colorSensorL.red()){
//                                beyonce.StrafeRight(0.5);
//                                //red = opModeIsActive() && 120 < colorSensorL.red();
//                                telemetry.addData("red", colorSensorL.red());
//                            }
//                            beyonce.Stop();
//                            telemetry.addData("Red Line 2", "Detected");

                            beyonce.DriveBackward(0.5);
                            sleep(700);
                            beyonce.Stop();

                            beyonce.DriveForward(0.5);
                            sleep(550);
                            beyonce.Stop();
                            sleep(500);
                            beyonce.ArmDown(-0.5);
                            sleep(1800);
                            beyonce.ArmDown(-0.25);
                            sleep(250);
                            beyonce.ClawOpen();
                            sleep(200);
                            beyonce.DriveForward(0.75);
                            sleep(400);
                            beyonce.Stop();

                            targetFound = true;
                        } else if (state == 1) { //if quad
                            telemetry.addData("quad:", "quad");

                            beyonce.ShooterOn();
                            sleep(2000);
                            beyonce.Shoot();
                            sleep(1000);
                            beyonce.Shoot();
                            sleep(1000);
                            beyonce.Shoot();
                            sleep(1000);
                            beyonce.ShooterOff();


                            beyonce.ClawClose();
                            beyonce.StrafeLeft(0.7);//i added this to straighten it out more
                            sleep(500);
                            beyonce.DriveBackward(0.6);
                            sleep(1000);
                            beyonce.StrafeRight(0.9);
                            sleep(4000);

                            beyonce.Stop();

                            //fowards until detect red
//                            while (opModeIsActive() && 120 > colorSensorL.red()){
//                                beyonce.DriveForward(0.3);
//                                //red = opModeIsActive() && 120 < colorSensorL.red();
//                                telemetry.addData("red", colorSensorL.red());
//                            }
//                            beyonce.Stop();
//                            telemetry.addData("Red Line", "Detected");

                            beyonce.DriveBackward(0.5);
                            sleep(500);
                            beyonce.Stop();
                            sleep(500);
                            beyonce.ArmDown(-0.5);
                            sleep(1800);
                            beyonce.ArmDown(-0.25);
                            sleep(250);
                            sleep(500);
                            beyonce.ClawOpen();
                            sleep(500);
                            beyonce.DriveForward(0.75);
                            sleep(250);
                            beyonce.StrafeLeft(1);
                            sleep(1300);
                            beyonce.Stop();
                            targetFound = true;
                        } else if (state == 2) { //if single
                            telemetry.addData("single", "single");

                            beyonce.ShooterOn();
                            sleep(2000);
                            beyonce.Shoot();
                            sleep(1000);
                            beyonce.Shoot();
                            sleep(1000);
                            beyonce.Shoot();
//                            sleep(1000);
                            beyonce.ShooterOff();
//

                            beyonce.ClawClose();
                            beyonce.DriveBackward(0.7);
                            sleep(800);
                        ;

                            while (opModeIsActive() && redVal > colorSensorL.red()){
                                beyonce.StrafeRight(0.7);
                                //red = opModeIsActive() && 120 < colorSensorL.red();
                                telemetry.addData("red", colorSensorL.red());
                            }
                            beyonce.Stop();
                            telemetry.addData("Red Line 1", "Detected");

                            beyonce.DriveBackward(0.7);
                            sleep(200);
                            beyonce.Stop();

                            beyonce.StrafeRight(0.8);
                            sleep(100);
                            beyonce.Stop();

                            while (opModeIsActive() && redVal > colorSensorL.red()){
                                beyonce.StrafeRight(0.8);
                                //red = opModeIsActive() && 120 < colorSensorL.red();
                                telemetry.addData("red", colorSensorL.red());
                            }
                            beyonce.Stop();
                            telemetry.addData("Red Line 2", "Detected");

//                            beyonce.DriveBackward(0.7);
//                            sleep(500);
//                            beyonce.Stop();

                            beyonce.StrafeRight(0.7);
                            sleep(300);
                            beyonce.Stop();

                            while (opModeIsActive() && redVal > colorSensorL.red()){
                                beyonce.StrafeRight(0.7);
                                //red = opModeIsActive() && 120 < colorSensorL.red();
                                telemetry.addData("red", colorSensorL.red());
                            }
                            beyonce.Stop();
                            telemetry.addData("Red Line 3", "Detected");

                            beyonce.DriveBackward(0.7);
                            sleep(750);


                            while (opModeIsActive() && redVal > colorSensorR.red()){
                                beyonce.DriveForward(0.7);
                                //red = opModeIsActive() && 120 < colorSensorL.red();
                                telemetry.addData("red", colorSensorR.red());
                            }
                            telemetry.addData("line", "detect");
                            beyonce.Stop();
                            sleep(500);

                            beyonce.DriveForward(0.5);
                            sleep(150);
                            beyonce.Stop();

//                            while (opModeIsActive() && redVal > colorSensorR.red()){
//                                beyonce.DriveForward(0.7);
//                                //red = opModeIsActive() && 120 < colorSensorL.red();
//                                telemetry.addData("red", colorSensorR.red());
//                            }
//                            telemetry.addData("line", "detect");
//                            beyonce.Stop();

                            sleep(500);
                            beyonce.ArmDown(-0.5);
                            sleep(1800);
                            beyonce.ArmDown(-0.25);
                            sleep(250);
                            sleep(500);
                            beyonce.ClawOpen();
                            sleep(500);
                            beyonce.DriveForward(0.75);
                            sleep(250);
                            beyonce.StrafeLeft(1);
                            sleep(700);
                            beyonce.Stop();
                            targetFound = true;
                        } else {
                            beyonce.ShooterOn();
                            sleep(2000);
                            beyonce.Shoot();
                            sleep(1000);
                            beyonce.Shoot();
                            sleep(1000);
                            beyonce.Shoot();
                            sleep(1000);
                            beyonce.ShooterOff();


                            beyonce.ClawClose();
                            beyonce.DriveBackward(0.5);
                            sleep(1500);

                            while (opModeIsActive() && redVal > colorSensorL.red()){
                                beyonce.StrafeRight(0.5);
                                //red = opModeIsActive() && 120 < colorSensorL.red();
                                telemetry.addData("red", colorSensorL.red());
                            }
                            beyonce.Stop();
                            telemetry.addData("Red Line 1", "Detected");

                            beyonce.DriveBackward(0.5);
                            sleep(200);
                            beyonce.Stop();

                            beyonce.StrafeRight(0.5);
                            sleep(200);
                            beyonce.Stop();

                            while (opModeIsActive() && redVal > colorSensorL.red()){
                                beyonce.StrafeRight(0.5);
                                //red = opModeIsActive() && 120 < colorSensorL.red();
                                telemetry.addData("red", colorSensorL.red());
                            }
                            beyonce.Stop();
                            telemetry.addData("Red Line 2", "Detected");

                            beyonce.DriveBackward(0.5);
                            sleep(700);
                            beyonce.Stop();

                            beyonce.DriveForward(0.5);
                            sleep(1200);
                            beyonce.Stop();
                            sleep(500);
                            beyonce.ArmDown(-0.5);
                            sleep(1800);
                            beyonce.ArmDown(-0.25);
                            sleep(250);
                            beyonce.ClawOpen();
                            sleep(200);
                            beyonce.DriveForward(0.75);
                            sleep(400);
                            beyonce.Stop();
                            beyonce.StrafeLeft(0.5);
                            sleep(1000);
                            beyonce.Stop();

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


}
