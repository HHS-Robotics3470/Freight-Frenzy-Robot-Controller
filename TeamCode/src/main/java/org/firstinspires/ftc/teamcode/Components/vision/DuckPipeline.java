package org.firstinspires.ftc.teamcode.Components.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * pipeline for OpenCV for the Freight Frenzy season
 * @author Anthony Rubick
 */
public class DuckPipeline extends OpenCvPipeline
{
    /* enums */
    //possible positions of the team game element
    public enum TSEPosition
    {
        LEFT,
        CENTER,
        RIGHT
    }

       /*
    camera resolution thing
    note, origin in top left corner of screen
    (0,0)------------------------------------------------------(1280,0)
      |                                                            |
      |                                                            |
      |                                                            |
      |                                                            |
      |                                                            |
      |                                                            |
      |                                                            |
      |                                                            |
      |                                                            |
      |                                                            |
      |                                                            |
      |                                                            |
    (0,960)---------------------------------------------------(1280,960)
     */

    /* general constants */
    //resolution of input stream
    public static final int resWidth = 1280;
    public static final int resHeight = 960;

    static final Scalar BLUE = new Scalar(0, 0, 255); //color blue
    static final Scalar GREEN = new Scalar(0, 255, 0); //color green


    /* Center and Right regions */
    static final Point CENTER_REGION_TOPLEFT_ANCHOR_POINT = new Point(540,310); //top left corner of the center box
    static final Point RIGHT_REGION_TOPLEFT_ANCHOR_POINT = new Point(1040,280); //top left corner of the right box
    static final int REGION_WIDTH = 200; //width of boxes
    static final int REGION_HEIGHT = 375; //height of boxes

    //top left and bottom right corners of center box
    Point center_region_pointA = new Point(
            CENTER_REGION_TOPLEFT_ANCHOR_POINT.x,
            CENTER_REGION_TOPLEFT_ANCHOR_POINT.y);
    Point center_region_pointB = new Point(
            CENTER_REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            CENTER_REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    //top left and bottom right corners of right box
    Point right_region_pointA = new Point(
            RIGHT_REGION_TOPLEFT_ANCHOR_POINT.x,
            RIGHT_REGION_TOPLEFT_ANCHOR_POINT.y);
    Point right_region_pointB = new Point(
            RIGHT_REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            RIGHT_REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);


    /*readings from image*/
    Mat center_region_Cb, right_region_Cb;//Cb values of the center and right regions of the input image
    Mat YCrCb = new Mat(); // the input image in the YCrCb color space
    Mat Cb = new Mat(); // the Cb channel of the YCrCb image (blue)

    private volatile TSEPosition position = TSEPosition.LEFT; // default position

    private volatile int rightAvg; //average reading from right region
    private volatile int centerAvg; //average reading from left region

    /* methods */
    /**
     * converts input to the YCrCb color space, and saves the new image to the YCrCb matrix
     * then extracts the Cb color channel from the YCrCb matrix and saves it to the Cb matrix
     * @param input input image in the RGB color space
     */
    private void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);    // last one is the color scheme
                                                                    // take the input, convert it to the YCrCb color space, and save the new image to the YCrCb matrix
        Core.extractChannel(YCrCb, Cb, 2); // take the contents of channel 2 from YCrCb and copy it into CB
                                                // take the Cb channel from YCrCb and copy it to the Cb matrix
    }

    /**
     * initialize the pipeline, don't need to call this anywhere,
     * it runs automatically
     * @param firstFrame the first frame that the camera sees
     */
    @Override
    public void init(Mat firstFrame)
    {
        inputToCb(firstFrame); // take the first frame, and convert it to the YCrCb color space, also extract the Cb channel from it

        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        center_region_Cb = Cb.submat(new Rect(center_region_pointA, center_region_pointB)); //this holds the Cb of pixels within the center box
        right_region_Cb = Cb.submat(new Rect(right_region_pointA, right_region_pointB)); //this holds the Cb of pixels within the right box
    }

    /**
     * runs in its own thread (pre-defined and managed by the library).
     * extracts and processes frames, returning whatever you want to show up on the viewport / camera stream.
     * @param input the image that the camera sees
     * @return the image shown on the viewport
     */
    @Override
    public Mat processFrame(Mat input)
    {
        inputToCb(input);// take the input frame, and convert it to the YCrCb color space, also extract the Cb channel from it

        centerAvg = (int) Core.mean(center_region_Cb).val[0]; // average the values of center_region_Cb, get a scalar (size 1), access the value at index 0 (average Cb)
        rightAvg = (int) Core.mean(right_region_Cb).val[0]; // average the values of right_region_Cb, get a scalar (size 1)access the value at index 0 (average Cb)

        //draw center box
        Imgproc.rectangle(
                input, //image the rectangle is drawn on
                center_region_pointA, // top left corner of image
                center_region_pointB, // bottom right corner of image
                BLUE, //color of border
                2); //thickness of border

        //draw right box
        Imgproc.rectangle(
                input,
                right_region_pointA,
                right_region_pointB,
                BLUE,
                2);

        //logic to determine the level
        if(centerAvg < 145)
        {
            position = TSEPosition.CENTER;

            //if its detected, draw the box green
            Imgproc.rectangle(
                    input,
                    center_region_pointA,
                    center_region_pointB,
                    GREEN,
                    -1);
        }
        else if(rightAvg < 130)
        {
            position = TSEPosition.RIGHT;

            //if its detected, draw the box green
            Imgproc.rectangle(
                    input,
                    right_region_pointA,
                    right_region_pointB,
                    GREEN,
                    -1);
        }
        else
        {
            position = TSEPosition.LEFT;
        }

        return input;
    }

    /**
     * reads from TSEPosition and returns the corresponding integer
     * @return integer representing the state of the position enum
     */
    public int getAnalysis()
    {
        if (position == TSEPosition.LEFT){
            return 0;
        }
        else if (position == TSEPosition.CENTER){
            return 1;
        }
        else if (position == TSEPosition.RIGHT){
            return 2;
        }
        return -1;
    }

    /**
     * @return average reading of center region
     */
    public int getCenterAvg()
    {
        return centerAvg;
    }

    /**
     * @return average reading of right region
     */
    public int getRightAvg()
    {
        return rightAvg;
    }

}