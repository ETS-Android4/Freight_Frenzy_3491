package org.firstinspires.ftc.teamcode.Season_Setup;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;


public class Freight_Frenzy_Pipeline extends Ducky
{

    // Variable Declaration
    public static volatile Pipeline.ElementPosition positionOfTeamShippingElement = Pipeline.ElementPosition.LEFT;

    /**
     * Pipeline Class
     */
    static class Pipeline extends OpenCvPipeline
    {
        // Camera View set-up
        boolean viewportPaused;
        public OpenCvCamera webcam;

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
        static final Point LEFT_BARCODE_TOP_LEFT_ANCHOR_POINT =   new Point(56, 100);
        static final Point CENTER_BARCODE_TOP_LEFT_ANCHOR_POINT = new Point(162, 100);
        static final Point RIGHT_BARCODE_TOP_LEFT_ANCHOR_POINT =  new Point(268, 100);

        static final int REGION_HEIGHT = 50;        static final int REGION_WIDTH = 40;


        // Threshold in which the Element will be detected
        final double ELEMENT_THRESHOLD = 0.5;

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
        public double leftValue;
        public double centerValue;
        public double rightValue;

        // Volatile since accessed by OpMode thread w/o synchronization
        Pipeline.ElementPosition position = ElementPosition.LEFT;

        /**
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToHSV(Mat input) {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
            Scalar lowHSV = new Scalar(200, 200, 200);
            Scalar highHSV = new Scalar(255, 255, 255);

            Core.inRange(mat, lowHSV, highHSV, mat);
        }

        /**
         * @param frame Initializing the frame
         */
        @Override
        public void init(Mat frame) {
            inputToHSV(frame);
            leftBarcode = mat.submat(LEFT_BARCODE);
            centerBarcode = mat.submat(CENTER_BARCODE);
            rightBarcode = mat.submat(RIGHT_BARCODE);
        }

        /**
         * @param input Processing the frames
         * @return Returns the Team Shipping Element Position
         */
        @Override
        public Mat processFrame(Mat input) {

            // Initializing the frame
            inputToHSV(input);

            // Setting Variable Value
            leftValue = Core.sumElems(leftBarcode).val[0];
            centerValue = Core.sumElems(centerBarcode).val[0];
            rightValue = Core.sumElems(rightBarcode).val[0];

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
            if (leftValue > ELEMENT_THRESHOLD) {
                elementPosition = ElementPosition.LEFT;
            } else if (centerValue > ELEMENT_THRESHOLD) {
                elementPosition = ElementPosition.CENTER;
            } else if (rightValue > ELEMENT_THRESHOLD) {
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