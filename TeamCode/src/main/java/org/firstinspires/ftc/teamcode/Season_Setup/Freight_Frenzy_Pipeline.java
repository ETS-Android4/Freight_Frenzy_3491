package org.firstinspires.ftc.teamcode.Season_Setup;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


public class Freight_Frenzy_Pipeline extends Ducky
{

    // Variable Declaration
    public static volatile Pipeline.ElementPosition positionOfTeamShippingElement = Pipeline.ElementPosition.LEFT;

    /**
     * Pipeline Class
     */
    public static class Pipeline extends OpenCvPipeline
    {
        // Camera View set-up
        boolean viewportPaused;
        public OpenCvCamera webcam;

        public double exposure_value;
//        exposure_value = OpenCvWebcam.getExposureControl();

        // Viewport setup
        @Override
        public void onViewportTapped()
        {
            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }


        // An enum to define the Team Shipping Element's position on the Barcode
        public enum ElementPosition {
            LEFT,
            CENTER,
            RIGHT
        }

        public ElementPosition elementPosition;

        // Color constant
        final Scalar WHITE = new Scalar(255, 255, 255);

        // The core values which define the location and size of the sample regions
        static final int REGION_HEIGHT = 100;        static final int REGION_WIDTH = 70;

        static final int CENTER_BARCODE_TOP_LEFT_X_ANCHOR_POINT = 320/2 - REGION_WIDTH/2;

        static final Point LEFT_BARCODE_TOP_LEFT_ANCHOR_POINT =   new Point(0, 100);
        static final Point CENTER_BARCODE_TOP_LEFT_ANCHOR_POINT = new Point(CENTER_BARCODE_TOP_LEFT_X_ANCHOR_POINT, 100);
        static final Point RIGHT_BARCODE_TOP_LEFT_ANCHOR_POINT =  new Point(320-REGION_WIDTH, 100);


        ////* Creating the sample regions *////
        /* pointA = The Top left point of the region */
        /* pointB = The Bottom right point of the region */

        // Left Barcode
        static final Rect LEFT_BARCODE = new Rect(
                new Point(LEFT_BARCODE_TOP_LEFT_ANCHOR_POINT.x,
                          LEFT_BARCODE_TOP_LEFT_ANCHOR_POINT.y),
                new Point(LEFT_BARCODE_TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH,
                          LEFT_BARCODE_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT));

        // Center Barcode
        static final Rect CENTER_BARCODE = new Rect(
                new Point(CENTER_BARCODE_TOP_LEFT_ANCHOR_POINT.x,
                          CENTER_BARCODE_TOP_LEFT_ANCHOR_POINT.y),
                new Point(CENTER_BARCODE_TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH,
                          CENTER_BARCODE_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT));

        // Right Barcode
        static final Rect RIGHT_BARCODE = new Rect(
                new Point(RIGHT_BARCODE_TOP_LEFT_ANCHOR_POINT.x,
                          RIGHT_BARCODE_TOP_LEFT_ANCHOR_POINT.y),
                new Point(RIGHT_BARCODE_TOP_LEFT_ANCHOR_POINT.x + REGION_WIDTH,
                          RIGHT_BARCODE_TOP_LEFT_ANCHOR_POINT.y + REGION_HEIGHT));

        // Variable Declaration
        Mat leftBarcode;
        Mat centerBarcode;
        Mat rightBarcode;
        Mat mat = new Mat();
        Mat vLeft = new Mat();
        Mat vCenter = new Mat();
        Mat vRight = new Mat();
        public double leftValue;
        public double centerValue;
        public double rightValue;

        /**
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToHSV(Mat input) {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
            Scalar lowHSV = new Scalar(79, 28, 30);
            Scalar highHSV = new Scalar(151, 100, 100);

            Core.inRange(mat, lowHSV, highHSV, mat);
        }

        /**
         * @param input Initializing the frame
         */
        @Override
        public void init(Mat input) {
            inputToHSV(input);
            leftBarcode = input.submat(LEFT_BARCODE);
            centerBarcode = input.submat(CENTER_BARCODE);
            rightBarcode = input.submat(RIGHT_BARCODE);
        }

        @Override
        public Mat processFrame(Mat input) {

            // Initializing the frame
            inputToHSV(input);

            // extract the v channel from hsv
            Core.extractChannel(leftBarcode, vLeft, 0);
            Core.extractChannel(centerBarcode, vCenter, 0);
            Core.extractChannel(rightBarcode, vRight, 0);

            // get the average colors
            leftValue = Core.mean(vLeft).val[0];
            centerValue = Core.mean(vCenter).val[0];
            rightValue = Core.mean(vRight).val[0];


            /* Drawing the Rectangles */
            // Left Barcode Rectangle
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    LEFT_BARCODE,
                    WHITE, // The colour of the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            // Center Barcode Rectangle
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    CENTER_BARCODE,
                    WHITE, // The colour of the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            // Right Barcode Rectangle
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    RIGHT_BARCODE,
                    WHITE, // The colour of the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            // Record out analysis
            if (leftValue > centerValue && leftValue > rightValue) {
                elementPosition = ElementPosition.LEFT;
            } else if (centerValue > leftValue && centerValue > rightValue) {
                elementPosition = ElementPosition.CENTER;
            } else if (rightValue > leftValue && rightValue > centerValue) {
                elementPosition = ElementPosition.RIGHT;
            } else {
                elementPosition = ElementPosition.LEFT;
            }

            // Setting Analysis Value for Telemetry
            analysisLeft = leftValue;
            analysisCenter = centerValue;
            analysisRight = rightValue;

            // Setting Position of Element for Telemetry
            positionOfTeamShippingElement = elementPosition;

            // Return Input
            return input;
        }
    }
}