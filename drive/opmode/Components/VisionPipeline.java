package org.firstinspires.ftc.teamcode.drive.opmode.Components;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisionPipeline extends OpenCvPipeline
{

    /*
     * An enum to define the skystone position
     */


    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);


    Scalar NEON_GREEN = new Scalar(30, 227, 39);

    // gray 135-160
    //black < 135
    //white > 160

    /*
     * The core values which define the location and size of the sample regions
     */


    static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(315,30);
    static int REGION_WIDTH = 175;
    static int REGION_HEIGHT = 240 ;


    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    /*
     * Working variables
     */
    Mat region1_Cb, region2_Cb, region3_Cb;
    Mat YCrCb = new Mat();
    Mat Y = new Mat();
    int avg1;

    final double thres_low = 135;
    final double thesh_high = 170;

    public enum sleeve_pos {one, two, three}

    public volatile sleeve_pos real_sleeve_pos = sleeve_pos.one;

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Y, 0);
    }

    @Override
    public void init(Mat firstFrame)
    {

        inputToCb(firstFrame);

        region1_Cb = Y.submat(new Rect(region1_pointA, region1_pointB));
    }

    @Override
    public Mat processFrame(Mat input)
    {
        inputToCb(input);

        avg1 = (int) Core.mean(region1_Cb).val[0];

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 2 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */


        if(avg1 < thres_low){
            real_sleeve_pos = sleeve_pos.three;
        }
        if(avg1 > thres_low && avg1 < thesh_high){
            real_sleeve_pos = sleeve_pos.two;
        }
        if(avg1 > thesh_high){
            real_sleeve_pos = sleeve_pos.one;
        }


        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return input;
    }

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    public sleeve_pos getAnalysis()
    {
        return real_sleeve_pos;
    }
    /*public int getPos(){
        int pos = 0;
        if (position == BarcodePosition.LEFT){
            pos = 1;
        }
        else if (position == BarcodePosition.CENTER){
            pos = 2;
        }
        else if (position == BarcodePosition.RIGHT){
            pos = 3;
        }
        return pos;
    }*/
    public int getAvg1(){
        return avg1;
    }

}