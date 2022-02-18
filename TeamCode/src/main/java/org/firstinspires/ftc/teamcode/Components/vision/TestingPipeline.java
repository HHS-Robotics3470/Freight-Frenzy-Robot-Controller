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
 * NOTES when transferring things from this pipeline to the main pipeline:
 *  -keep in mind that a lot of stuff is unoptimized to increase amount of available data, in practice DO NOT store any data that isn't used (this includes color channels of regions that aren't read from)
 * @author Anthony Rubick
 */
public class TestingPipeline extends OpenCvPipeline
{
    /* enums */
    //possible positions of the team game element
    public enum TSEPosition
    {
        LEFT,
        CENTER,
        RIGHT
    }

    /**
     * what view is returned to the viewport
     */
    private enum Viewtype {
        FULL_COLOR, //standard, shows what the camera sees with overlay
        Y_MASK, //shows the Y channel of what the camera sees in the YCrCb color space
        Cr_MASK, //shows the Cr channel of what the camera sees in the YCrCb color space
        Cb_MASK //shows the Cb channel of what the camera sees in the YCrCb color space
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

    /*
    TODO idea: last used in this commit: https://github.com/HHS-Robotics3470/FtcRobotController/commit/fe68e4207e262c202aabaaab96b2bdd192230506
     1 Add a third region below the current 2,
     2 from this read ambient average Y value
     3 then subtract that from the average Y of the 2 other regions
     4 if one of the remaining Y is significantly higher than the other, that's where the Team Scoring Element is
          this works because our TSE is a lighter color (white) than the surrounding area (gray),
          and should remain so in pretty much any lighting
       otherwise, if both Y's are relatively similar, then the TSE in the left position
      .
      if this doesn't work, just tune the current logic ig
     .
     Note:
          compares Y channel because our element is white, and therefore significantly brighter than the mats
          if it was yellow/blue, compare Cb channel
          if it was green/red, compare Cr channel
          .
          also note that some of the detection logic will need to be changed for those examples
    */
    /*
    TODO Idea 2:
     have 3 boxes, one for each barcode, and find the one with the most abnormal Y/Cr/Cb value (maybe compared to background but might not be needed)
     Note:
          compares Y channel because our element is white, and therefore significantly brighter than the mats
          if it was yellow/blue, compare Cb channel
          if it was green/red, compare Cr channel
          .
          also note that some of the detection logic will need to be changed for those examples
    */

    /* general constants */
    //resolution of input stream
    public static final int resWidth = 1280;
    public static final int resHeight = 960;

    static final Scalar BLUE = new Scalar(0, 0, 255); //color blue
    static final Scalar GREEN = new Scalar(0, 255, 0); //color green

    private Viewtype viewToRenderToViewport = Viewtype.FULL_COLOR;
    private final Viewtype[] views = Viewtype.values();


    /* Center and Right regions */
    //rectangle variables
    static final Point LEFT_REGION_TOPLEFT_ANCHOR_POINT = new Point(0,280); //top left corner of the left box
    static final Point CENTER_REGION_TOPLEFT_ANCHOR_POINT = new Point(500,310); //top left corner of the center box
    static final Point RIGHT_REGION_TOPLEFT_ANCHOR_POINT = new Point(1040,280); //top left corner of the right box
    static final int REGION_WIDTH = 240; //width of boxes
    static final int REGION_HEIGHT = 375; //height of boxes
    //top left and bottom right corners of left box
    Point leftRegionPointA = new Point(
            LEFT_REGION_TOPLEFT_ANCHOR_POINT.x,
            LEFT_REGION_TOPLEFT_ANCHOR_POINT.y);
    Point leftRegionPointB = new Point(
            LEFT_REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            LEFT_REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    //top left and bottom right corners of center box
    Point centerRegionPointA = new Point(
            CENTER_REGION_TOPLEFT_ANCHOR_POINT.x,
            CENTER_REGION_TOPLEFT_ANCHOR_POINT.y);
    Point centerRegionPointB = new Point(
            CENTER_REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            CENTER_REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    //top left and bottom right corners of right box
    Point rightRegionPointA = new Point(
            RIGHT_REGION_TOPLEFT_ANCHOR_POINT.x,
            RIGHT_REGION_TOPLEFT_ANCHOR_POINT.y);
    Point rightRegionPointB = new Point(
            RIGHT_REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            RIGHT_REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    //actual rectangles
    Rect leftRectangle = new Rect(leftRegionPointA,leftRegionPointB);
    Rect centerRectangle = new Rect(centerRegionPointA, centerRegionPointB);
    Rect rightRectangle = new Rect(rightRegionPointA, rightRegionPointB);


    /*readings from image*/
    Mat YCrCb = new Mat(); // the input image in the YCrCb color space
    Mat left_region, center_region, right_region; // various regions of YCrCb
    Mat leftRGB, centerRGB, rightRGB; //regions in rbg color space, used for detailed feedback (drawing the color it sees)

    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile TSEPosition position = TSEPosition.LEFT; // default position

    //scalar [ Y, Cr, Cb ]
    //       [ 0,  1,  2 ]
    //average reading from left region
    private volatile Scalar leftAvg = new Scalar(0,0,0);
    //average reading from center region
    private volatile Scalar centerAvg = new Scalar(0,0,0);
    //average readings from right region
    private volatile Scalar rightAvg = new Scalar(0,0,0);


    /* methods */
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
     * initialize the pipeline, don't need to call this anywhere,
     * it runs automatically
     * @param firstFrame the first frame that the camera sees
     */
    @Override
    public void init(Mat firstFrame)
    {
        extractChannels(firstFrame);


        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        left_region = YCrCb.submat(leftRectangle); //holds the YCrCb values of the pixels within the left box
        center_region = YCrCb.submat(centerRectangle); //holds the YCrCb values of the pixels within the center box
        right_region = YCrCb.submat(rightRectangle); //holds the YCrCb values of the pixels within the right box

        leftRGB = firstFrame.submat(leftRectangle);
        centerRGB = firstFrame.submat(centerRectangle);
        rightRGB = firstFrame.submat(rightRectangle);
    }

    /**
     * cycle the value assigned to viewToRenderToViewport
     * run in the UI thread
     * runs automatically when the viewport is tapped
     */
    public void cycleViewport()
    {
        /*
         * Note that this method is invoked from the UI thread
         * so whatever we do here, we must do quickly.
         */

        int currentViewNum = viewToRenderToViewport.ordinal();

        int nextViewNum = currentViewNum + 1;

        if(nextViewNum >= views.length)
        {
            nextViewNum = 0;
        }

        viewToRenderToViewport = views[nextViewNum];
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
        extractChannels(input);// take the input frame, and convert it to the YCrCb color space, also extract the Cb channel from it

        //scalar [ Y, Cr, Cb ]
        //       [ 0,  1,  2 ]
        leftAvg  = Core.mean(left_region); // average YCrCb values of left_region
        centerAvg = Core.mean(center_region); // average YCrCb values of center_region
        rightAvg  = Core.mean(right_region); // average YCrCb values of right_region

        //scalar [ Y, Cr, Cb ]
        //       [ 0,  1,  2 ]
        //average reading from left region
        Scalar leftAvgRGB = Core.mean(leftRGB);
        //average reading from center region
        Scalar centerAvgRGB = Core.mean(centerRGB);
        //average readings from right region
        Scalar rightAvgRGB = Core.mean(rightRGB);

        //draw left box
        drawDetailedBorderAroundRegion(
                input,
                leftRegionPointA,
                leftRegionPointB,
                leftAvgRGB,
                BLUE
        );//draw center box
        drawDetailedBorderAroundRegion(
                input,
                centerRegionPointA,
                centerRegionPointB,
                centerAvgRGB,
                BLUE
        );
        //draw right box
        drawDetailedBorderAroundRegion(
                input,
                rightRegionPointA,
                rightRegionPointB,
                rightAvgRGB,
                BLUE
        );


        //idea 1 logic
        //logic to determine the level
        int centerYDist = 0;
        int rightYDist = 0;
        if ( 0.75*centerYDist > rightYDist ) {// if center is more than 25% brighter than right compared to the bottom and top, todo: tune the value
            position = TSEPosition.CENTER;
            //if its detected, draw the box green
            drawDetailedBorderAroundRegion(
                    input,
                    centerRegionPointA,
                    centerRegionPointB,
                    centerAvgRGB,
                    GREEN
            );
        }
        else if ( 0.75*rightYDist > centerYDist ) {// if right is more than 25% brighter than center compared to the bottom and top, todo: tune the value
            position = TSEPosition.RIGHT;
            //if its detected, draw the box green
            drawDetailedBorderAroundRegion(
                    input,
                    rightRegionPointA,
                    rightRegionPointB,
                    rightAvgRGB,
                    GREEN
            );
        }
        else position = TSEPosition.LEFT;


        //return view
        switch (viewToRenderToViewport) {
            case Y_MASK:
                Mat Y = new Mat();
                Core.extractChannel(YCrCb, Y, 0);
                return Y;
            case Cr_MASK:
                Mat Cr = new Mat();
                Core.extractChannel(YCrCb, Cr, 1);
                return Cr;
            case Cb_MASK:
                Mat Cb = new Mat();
                Core.extractChannel(YCrCb, Cb, 2);
                return Cb;
            case FULL_COLOR:
            default:
                return input;
        }
    }

    /**
     * draws a border around a {region}
     * the border will be a {borderWidth} thick with the middle half of it being {innerColor} and the surrounding quarters being {outerColor}
     * @param input image to draw on
     * @param regionPointA top left corner of region
     * @param regionPointB bottom right corner of region
     * @param innerColor color of inner region of border
     * @param outerColor color of outer region of border
     */
    private void drawDetailedBorderAroundRegion(Mat input, Point regionPointA, Point regionPointB, Scalar innerColor, Scalar outerColor) {
        //outer border
        Imgproc.rectangle(
                input,
                regionPointA,
                regionPointB,
                outerColor,
                14
        );
        //middle
        Imgproc.rectangle(
                input,
                regionPointA,
                regionPointB,
                innerColor,
                10
        );
        //inner border
        /*Imgproc.rectangle(
                input,
                regionPointA,
                regionPointB,
                outerColor,
                4
        );
         */
    }

    private TSEPosition oddOneOut(double tolerance) {
        return TSEPosition.LEFT;
    }


    /* get methods*/
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
     * @return average reading of left region
     */
    public Scalar getLeftAvg() {
        return leftAvg;
    }

    /**
     * @return average reading of center region
     */
    public Scalar getCenterAvg()
    {
        return centerAvg;
    }

    /**
     * @return average reading of right region
     */
    public Scalar getRightAvg()
    {
        return rightAvg;
    }

    /**
     * @return string representation of view being rendered to viewport
     */
    public String getViewAsString() {
        return viewToRenderToViewport.toString();
    }
}