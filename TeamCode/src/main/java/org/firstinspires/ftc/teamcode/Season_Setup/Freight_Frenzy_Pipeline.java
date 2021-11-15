//package org.firstinspires.ftc.teamcode.Season_Setup;
//
//import android.annotation.SuppressLint;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//@TeleOp
//public class Freight_Frenzy_Pipeline extends Ducky
//{
//
//    public static volatile Pipeline.RingPosition positionOfRing = Pipeline.RingPosition.FOUR;
//
//
//    static class Pipeline extends OpenCvPipeline
//    {
//        // Camera View set-up
//        boolean viewportPaused;
//        public OpenCvCamera webcam;
//
//        @Override
//        public void onViewportTapped()
//        {
//            /*
//             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
//             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
//             * when you need your vision pipeline running, but do not require a live preview on the
//             * robot controller screen. For instance, this could be useful if you wish to see the live
//             * camera preview as you are initializing your robot, but you no longer require the live
//             * preview after you have finished your initialization process; pausing the viewport does
//             * not stop running your pipeline.
//             *
//             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
//             */
//            viewportPaused = !viewportPaused;
//
//            if(viewportPaused)
//            {
//                webcam.pauseViewport();
//            }
//            else
//            {
//                webcam.resumeViewport();
//            }
//        }
//
//        // Ring Detection set-up
//
//        // An enum to define the skystone position
//        public enum RingPosition {
//            FOUR,
//            ONE,
//            NONE
//        }
//
//        // Some color constants
//        final Scalar BLUE = new Scalar(0, 0, 255);
//
//        // The core values which define the location and size of the sample regions
//        final Point REGION1_TOP_LEFT_ANCHOR_POINT = new Point(181, 90);
//
//        static final int REGION_HEIGHT = 25;        static final int REGION_WIDTH = 35;
//
//
//        final int FOUR_RING_THRESHOLD = 150;
//        final int ONE_RING_THRESHOLD = 135;
//
//        Point region1_pointA = new Point(
//                REGION1_TOP_LEFT_ANCHOR_POINT.x,
//                REGION1_TOP_LEFT_ANCHOR_POINT.y);
//        Point region1_pointB = new Point(
//                REGION1_TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION1_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//        // Working variables
//        Mat region1_Cb;
//        Mat YCrCb = new Mat();
//        Mat Cb = new Mat();
//        int avg1;
//
//        // Volatile since accessed by OpMode thread w/o synchronization
//        RingPosition position = RingPosition.FOUR;
//
//        /*
//         * This function takes the RGB frame, converts to YCrCb,
//         * and extracts the Cb channel to the 'Cb' variable
//         */
//        void inputToCb(Mat input) {
//            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//            Core.extractChannel(YCrCb, Cb, 1);
//        }
//
//        @Override
//        public void init(Mat firstFrame) {
//            inputToCb(firstFrame);
//            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
//        }
//
//        /*
//         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
//         * highly recommended to declare them here as instance variables and re-use them for
//         * each invocation of processFrame(), rather than declaring them as new local variables
//         * each time through processFrame(). This removes the danger of causing a memory leak
//         * by forgetting to call mat.release(), and it also reduces memory pressure by not
//         * constantly allocating and freeing large chunks of memory.
//         */
//        @Override
//        public Mat processFrame(Mat input) {
//
//            inputToCb(input);
//
//            avg1 = (int) Core.mean(region1_Cb).val[0];
//
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    BLUE, // The colour of the rectangle is drawn in
//                    2); // Thickness of the rectangle lines
//
//            // Record out analysis
//            if (avg1 > FOUR_RING_THRESHOLD) {
//                position = RingPosition.FOUR;
//            } else if (avg1 > ONE_RING_THRESHOLD) {
//                position = RingPosition.ONE;
//            } else {
//                position = RingPosition.NONE;
//            }
//
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    BLUE, // The colour of the rectangle is drawn in
//                    -1); // Thickness of the rectangle lines
//
//            analysis = avg1;
//            positionOfRing = position;
//
//            return input;
//        }
//    }
//}

/*
 * Program is a mix of two different programs, one based on the example program shown in
 * "https://www.youtube.com/watch?v=-QFoOCoaW7I", By team "Wizards.exe 9794".
 *
 * The other is a program copied from the EasyOpenCV example library:
 * "https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/openftc/easyopencv/examples/WebcamExample.java"
 *
 * Program was adapted, modified, and changed to fix errors and to test and experiment in
 * EasyOpenCv.
 *
 * This program is able to used the Control Hub and Webcam set-up, to detect the ring stack from
 * the Ultimate Goal 2020-2021 season.
 */