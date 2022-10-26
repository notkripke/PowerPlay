package org.firstinspires.ftc.teamcode.drive.opmode.Components;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ConeDetectionPipeline extends OpenCvPipeline {

    public double pole_width;
    public double pole_center;

    @Override
    public Mat processFrame(Mat input) {

        Scalar upperBound = new Scalar(255, 100, 100);//Bounds for identifying red color
        Scalar lowerBound = new Scalar(178, 0, 0);

        Mat filter_frame1 = new Mat();

        Core.inRange(input, lowerBound, upperBound, filter_frame1);//create mask for yellow color
        input.setTo(new Scalar(255,255,255), filter_frame1);//yellow is changed to pure white

        Mat gray_frame = new Mat();

        Imgproc.cvtColor(input, gray_frame, Imgproc.COLOR_RGB2GRAY);//turn image grayscale

        Mat thresh_frame = new Mat();

        //turn image into binary
        Imgproc.threshold(gray_frame, thresh_frame, 250, 255, Imgproc.THRESH_BINARY);

        List<MatOfPoint> contours = new ArrayList<>();
        List<MatOfPoint> largest = new ArrayList<>();
        Mat hierarchey = new Mat();

        //find all contours in image
        Imgproc.findContours(thresh_frame, contours, hierarchey, Imgproc.RETR_TREE,
                Imgproc.CHAIN_APPROX_SIMPLE);

        double maxVal = 0;//Identify largest contour
        int maxValIdx = 0;
        for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++)
        {
            double contourArea = Imgproc.contourArea(contours.get(contourIdx));
            if (maxVal < contourArea)
            {
                maxVal = contourArea;
                maxValIdx = contourIdx;
                largest.clear();
                largest.add(contours.get(contourIdx));//select largest contour
            }
        }
        Mat contour_frame = new Mat();

        //Imgproc.drawContours(contour_frame, contours, maxValIdx, new Scalar(0,255,0), 5);

        //find center of contour
        List<Moments> mu = new ArrayList<Moments>(largest.size());
        for (int i = 0; i < largest.size(); i++) {
            mu.add(i, Imgproc.moments(largest.get(i), false));
            Moments p = mu.get(i);
            int x = (int) (p.get_m10() / p.get_m00());
            int y = (int) (p.get_m01() / p.get_m00());
            pole_center = x;
            //draw center of contour
            //Imgproc.circle(contour_frame, new Point(x,y), 4, new Scalar(0,255,0), 3);
            pole_width = Imgproc.boundingRect(contour_frame).width;
            //Core.circle(rgbaImage, new Point(x, y), 4, new Scalar(255,49,0,255));//draw center of pole in frame
        }

        return null;
    }

    public double getPoleWidth(){
        return pole_width;
    }
    public double getPoleCenter(){
        return  pole_center;
    }

}