package org.firstinspires.ftc.teamcode.Components.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * like the main pipeline (DuckPipeline.java), but used to test new detection logic, also gives more information
 * also: used by openCVTester.java
 *
 * NOTE: when transfering things from this pipeline to the main, keep in mind that a lot of stuff is unoptimized to increase amount of available data,
 * in practice DO NOT store any data that isn't used (this includes color channels of regions that aren't read from)
 */
public class TestingPipeline extends OpenCvPipeline
{
    //possible positions of the team game element
    public enum TSEPosition
    {
        LEFT,
        CENTER,
        RIGHT
    }


    //TODO idea:
    // 1 Add a third region below the current 2,
    // 2 from this read ambient average Y value
    // 3 then subtract that from the average Y of the 2 other regions
    // 4 if one of the remaining Y is significantly higher than the other, that's where the Team Scoring Element is
    //      this works because our TSE is a lighter color (white) than the surrounding area (gray),
    //      and should remain so in pretty much any lighting
    //   otherwise, if both Y's are relatively similar, then the TSE in the left position
    //  .
    //  if this doesn't work, just tune the current logic ig

    //resolution of input stream
    public static final int resWidth = 1280;
    public static final int resHeight = 960;

    static final Scalar BLUE = new Scalar(0, 0, 255); //color blue
    static final Scalar GREEN = new Scalar(0, 255, 0); //color green

    //rectangle variables
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
    //actual rectangles
    Rect centerRectangle = new Rect(center_region_pointA, center_region_pointB);
    Rect rightRectangle = new Rect(right_region_pointA, right_region_pointB);

    //matrixes
    Mat center_region, right_region; // center and right regions of the input image
    Mat YCrCb = new Mat(); // the input image in the YCrCb color space
    //center and right boxes

    private volatile TSEPosition position = TSEPosition.LEFT; // default position

    //scalar [ Y, Cr, Cb ]
    //       [ 0,  1,  2 ]
    //average reading from center box
    private volatile Scalar centerAvg;
    //average readings from right box
    private volatile Scalar rightAvg;

    //scalar [ Y, Cr, Cb ]
    //       [ 0,  1,  2 ]
    //average reading from center box
    private volatile Scalar centerAvgRGB;
    //average readings from right box
    private volatile Scalar rightAvgRGB;

    /**
     * converts input to the YCrCb color space, and saves the new image to the YCrCb matrix
     * then extracts the color channels from the YCrCb matrix and saves them to their corresponding matrix's
     * @param input input image in the RGB color space
     */
    private void extractChannels(Mat input) {
        // take the input, convert it to the YCrCb color space, and save the new image to the YCrCb array
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
    }

    /**
     * initialize the pipeline
     * @param firstFrame the first frame that the camera sees
     */
    @Override
    public void init(Mat firstFrame)
    {
        extractChannels(firstFrame);

        center_region = YCrCb.submat(centerRectangle); //holds the YCrCb values of the pixels within the center box
        right_region = YCrCb.submat(rightRectangle); //holds the YCrCb values of the pixels within the center box
    }

    /**
     * seems like this is run asynchronously or something, and we access the results of this through other methods like getAnalysis()
     * kinda like the runOpMode of an opMode
     * @param input the image that the camera sees
     * @return the input, not sure why
     */
    @Override
    public Mat processFrame(Mat input)
    {
        extractChannels(input);// take the input frame, and convert it to the YCrCb color space, also extract the Cb channel from it

        //scalar [ Y, Cr, Cb ]
        //       [ 0,  1,  2 ]
        centerAvg = Core.mean(center_region); // average YCrCb values of center_region
        rightAvg  = Core.mean(right_region); // average YCrCb values of right_region

        centerAvgRGB = Core.mean(input.submat(centerRectangle));
        rightAvgRGB  = Core.mean(input.submat(rightRectangle));

        //draw center box
        //outer border, 1 pix
        Imgproc.rectangle(
                input, //image the rectangle is drawn on
                center_region_pointA, // top left corner of image
                center_region_pointB, // bottom right corner of image
                BLUE, //color of border
                4); //thickness of border
        //middle, 3 pixels wide
        Imgproc.rectangle(
                input,
                center_region_pointA,
                center_region_pointB,
                centerAvgRGB, //color being detected in middle box
                3);
        //inner border, 1 pix
        Imgproc.rectangle(
                input,
                center_region_pointA,
                center_region_pointB,
                BLUE,
                -1);

        //draw right box
        //outer border, 1 pix
        Imgproc.rectangle(
                input,
                right_region_pointA,
                right_region_pointB,
                BLUE,
                4);
        //middle, 3 pixels wide
        Imgproc.rectangle(
                input,
                right_region_pointA,
                right_region_pointB,
                rightAvgRGB, //color being detected in right box
                3);
        //inner border, 1 pix
        Imgproc.rectangle(
                input,
                right_region_pointA,
                right_region_pointB,
                BLUE,
                -1);

        //logic to determine the level
        if((int) centerAvg.val[2] < 145)
        {
            position = TSEPosition.CENTER;

            //if its detected, draw the box green
            Imgproc.rectangle(
                    input,
                    center_region_pointA,
                    center_region_pointB,
                    GREEN,
                    -2);
        }
        else if((int) rightAvg.val[2] < 130)
        {
            position = TSEPosition.RIGHT;

            //if its detected, draw the box green
            Imgproc.rectangle(
                    input,
                    right_region_pointA,
                    right_region_pointB,
                    GREEN,
                    -2);
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
     * @return average reading of center box
     */
    public Scalar getCenterAvg()
    {
        return centerAvg;
    }

    /**
     * @return average reading of right box
     */
    public Scalar getRightAvg()
    {
        return rightAvg;
    }

}