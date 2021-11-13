//package org.firstinspires.ftc.teamcode.Season_Setup;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//@TeleOp
//public class Freight_Frenzy_Pipeline_Proper extends Ducky
//{
//
//    // Variable Declaration
//    public static volatile Pipeline.ElementPosition positionOfTeamShippingElement = Pipeline.ElementPosition.LEFT;
//
//    /**
//     * Pipeline Class
//     */
//    static class Pipeline extends OpenCvPipeline
//    {
//        // Camera View set-up
//        boolean viewportPaused;
//        public OpenCvCamera webcam;
//
//        // Viewport setup
//        @Override
//        public void onViewportTapped()
//        {
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
//
//        // An enum to define the Team Shipping Element's position on the Barcode
//        public enum ElementPosition {
//            LEFT,
//            CENTER,
//            RIGHT
//        }
//
//        // Color constant
//        final Scalar WHITE = new Scalar(255, 255, 255);
//
//        // The core values which define the location and size of the sample regions
//        final Point LEFT_BARCODE_TOP_LEFT_ANCHOR_POINT =   new Point(56, 100);
//        final Point CENTER_BARCODE_TOP_LEFT_ANCHOR_POINT = new Point(162, 100);
//        final Point RIGHT_BARCODE_TOP_LEFT_ANCHOR_POINT =  new Point(268, 100);
//
//        static final int REGION_HEIGHT = 50;        static final int REGION_WIDTH = 40;
//
//
//        // Threshold in which the Element will be detected
//        final int ELEMENT_THRESHOLD = 150;
//
//        ////* Creating the sample regions *////
//        /* pointA = The Top left point of the region */
//        /* pointB = The Bottom right point of the region */
//
//        // Left Barcode
//        Point Left_Barcode_pointA = new Point(
//                LEFT_BARCODE_TOP_LEFT_ANCHOR_POINT.x,
//                LEFT_BARCODE_TOP_LEFT_ANCHOR_POINT.y);
//        Point Left_Barcode_pointB = new Point(
//                LEFT_BARCODE_TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                LEFT_BARCODE_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//        // Center Barcode
//        Point Center_Barcode_pointA = new Point(
//                CENTER_BARCODE_TOP_LEFT_ANCHOR_POINT.x,
//                CENTER_BARCODE_TOP_LEFT_ANCHOR_POINT.y);
//        Point Center_Barcode_pointB = new Point(
//                CENTER_BARCODE_TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                CENTER_BARCODE_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//        // Right Barcode
//        Point Right_Barcode_pointA = new Point(
//                RIGHT_BARCODE_TOP_LEFT_ANCHOR_POINT.x,
//                RIGHT_BARCODE_TOP_LEFT_ANCHOR_POINT.y);
//        Point Right_Barcode_pointB = new Point(
//                RIGHT_BARCODE_TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                RIGHT_BARCODE_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//        // Variable Declaration
//        Mat leftBarcode;
//        Mat centerBarcode;
//        Mat rightBarcode;
//        Mat YCrCb = new Mat();
//        Mat Cb = new Mat();
//        int avg1;
//        int avg2;
//        int avg3;
//
//        // Volatile since accessed by OpMode thread w/o synchronization
//        Pipeline.ElementPosition position = ElementPosition.LEFT;
//
//        /**
//         * This function takes the RGB frame, converts to YCrCb,
//         * and extracts the Cb channel to the 'Cb' variable
//         */
//        void inputToCb(Mat input) {
//            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//            Core.extractChannel(YCrCb, Cb, 1);
//        }
//
//        /**
//         * @param frame Initializing the frame
//         */
//        @Override
//        public void init(Mat frame) {
//            inputToCb(frame);
//            leftBarcode = Cb.submat(new Rect(Left_Barcode_pointA, Left_Barcode_pointB));
//            centerBarcode = Cb.submat(new Rect(Center_Barcode_pointA, Center_Barcode_pointB));
//            rightBarcode = Cb.submat(new Rect(Right_Barcode_pointA, Right_Barcode_pointB));
//        }
//
//        /**
//         * @param input Processing the frames
//         * @return Returns the Team Shipping Element Position
//         */
//        @Override
//        public Mat processFrame(Mat input) {
//
//            // Initializing the frame
//            inputToCb(input);
//
//            // Setting Variable Value
//            avg1 = (int) Core.mean(leftBarcode).val[0];
//            avg2 = (int) Core.mean(centerBarcode).val[0];
//            avg3 = (int) Core.mean(rightBarcode).val[0];
//
//            // Left Barcode Rectangle
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    Left_Barcode_pointA, // First point which defines the rectangle
//                    Left_Barcode_pointB, // Second point which defines the rectangle
//                    WHITE, // The colour of the rectangle is drawn in
//                    1); // Thickness of the rectangle lines
//
//            // Center Barcode Rectangle
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    Center_Barcode_pointA, // First point which defines the rectangle
//                    Center_Barcode_pointB, // Second point which defines the rectangle
//                    WHITE, // The colour of the rectangle is drawn in
//                    1); // Thickness of the rectangle lines
//
//            // Right Barcode Rectangle
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    Right_Barcode_pointA, // First point which defines the rectangle
//                    Right_Barcode_pointB, // Second point which defines the rectangle
//                    WHITE, // The colour of the rectangle is drawn in
//                    1); // Thickness of the rectangle lines
//
//            // Record out analysis
//            if (avg1 > ELEMENT_THRESHOLD) {
//                position = ElementPosition.LEFT;
//            } else if (avg2 > ELEMENT_THRESHOLD) {
//                position = ElementPosition.CENTER;
//            } else if (avg3 > ELEMENT_THRESHOLD) {
//                position = ElementPosition.RIGHT;
//            }
//
//            // Left Barcode Rectangle
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    Left_Barcode_pointA, // First point which defines the rectangle
//                    Left_Barcode_pointB, // Second point which defines the rectangle
//                    WHITE, // The colour of the rectangle is drawn in
//                    -1); // Thickness of the rectangle lines
//
//            // Center Barcode Rectangle
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    Center_Barcode_pointA, // First point which defines the rectangle
//                    Center_Barcode_pointB, // Second point which defines the rectangle
//                    WHITE, // The colour of the rectangle is drawn in
//                    -1); // Thickness of the rectangle lines
//
//            // Right Barcode Rectangle
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    Right_Barcode_pointA, // First point which defines the rectangle
//                    Right_Barcode_pointB, // Second point which defines the rectangle
//                    WHITE, // The colour of the rectangle is drawn in
//                    -1); // Thickness of the rectangle lines
//
//            // Setting Variable value
//            analysis = avg1;
//            positionOfTeamShippingElement = position;
//
//            return input;
//        }
//    }
//}