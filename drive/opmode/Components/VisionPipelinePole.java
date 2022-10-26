package org.firstinspires.ftc.teamcode.drive.opmode.Components;

import android.graphics.Bitmap;
import android.util.Log;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class VisionPipelinePole extends OpenCvPipeline {

    Scalar upperBound = new Scalar(252, 177, 3);//Bounds for identifying yellow color
    Scalar lowerBound = new Scalar(252, 235, 3);

    Mat filter_frame1 = new Mat();
    Mat filter_frame2 = new Mat();

    Mat gray_frame = new Mat();

    @Override
    public Mat processFrame(Mat input) {

        double xBlack, xBlackCenter, yBlack, battHeight;

        Bitmap roiBitmap = null;
        Scalar green = new Scalar(0, 255, 0, 255);
        Mat sourceMat = new Mat(1280, 720, CvType.CV_8UC3);

        Imgproc.blur(input, sourceMat, new Size(20,30));//50, 70

        Mat roiTmp = sourceMat.clone();
        //bitmapWidth = sourceBitmap.getWidth();
        //Log.e("bitmapWidth", String.valueOf(1280));
        final Mat hsvMat = new Mat();
        //sourceMat.copyTo(hsvMat);

        // convert mat to HSV format for Core.inRange()
        Imgproc.cvtColor(roiTmp, hsvMat, Imgproc.COLOR_RGB2HSV);


        Scalar lowerblack = new Scalar(41, 88, 78);         // lower color border for BLACK 100 100 100
        Scalar upperblack = new Scalar(56, 100, 100);      // upper color border for BLACK 180 255 255

        //^WORKING FOR RED+BLUE: 100, 80, 80 - - -  255, 225, 225
        //RED CONE ONLY: 150, 80, 80 - - - 255, 225, 225

        Scalar testRunL = new Scalar(70, 30, 0); // lower Green   83 100 51
        Scalar testRunU = new Scalar(255, 255, 255); // upper Green

        Core.inRange(hsvMat, lowerblack, upperblack, roiTmp);   // select only blue pixels
        // find contours
        List<MatOfPoint> contours = new ArrayList<>();
        List<RotatedRect> boundingRects = new ArrayList<>();
        Imgproc.findContours(roiTmp, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        // find appropriate bounding rectangles
        for (MatOfPoint contour : contours) {
            MatOfPoint2f areaPoints = new MatOfPoint2f(contour.toArray());
            RotatedRect boundingRect = Imgproc.minAreaRect(areaPoints);


            double rectangleArea = boundingRect.size.area();

            // test min ROI area in pixels
            if (rectangleArea > 3000 && rectangleArea < 800000) {//400000
                Point rotated_rect_points[] = new Point[4];
                boundingRect.points(rotated_rect_points);
                Rect rect3 = Imgproc.boundingRect(new MatOfPoint(rotated_rect_points));

                Log.e("blackArea", String.valueOf(rect3.area()));
                // test horizontal ROI orientation
                if (rect3.height > rect3.width) {
                    Imgproc.rectangle(sourceMat, rect3.tl(), rect3.br(), green, 3);
                    xBlack = rect3.br().x;
                    xBlackCenter = (rect3.br().x + rect3.tl().x) / 2;
                    yBlack = rect3.br().y;//bottom
                    battHeight = (rect3.br().y - rect3.tl().y); //batt height in pixel
                    Log.e("BLACKBR, TL", String.valueOf(rect3.br().y) + "," + String.valueOf(rect3.tl().y));
                }

            }

        }
        roiBitmap = Bitmap.createBitmap(sourceMat.cols(), sourceMat.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(sourceMat, roiBitmap);

        //return sourceMat;
        return roiTmp;


}

}


/*
package org.firstinspires.ftc.teamcode.drive.opmode.Components;

import android.graphics.Bitmap;
import android.util.Log;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class VisionPipelinePole extends OpenCvPipeline {

    Scalar upperBound = new Scalar(252, 177, 3);//Bounds for identifying yellow color
    Scalar lowerBound = new Scalar(252, 235, 3);

    Mat filter_frame1 = new Mat();
    Mat filter_frame2 = new Mat();

    Mat gray_frame = new Mat();

    @Override
    public Mat processFrame(Mat input) {

        double xBlack, xBlackCenter, yBlack, battHeight;

        Bitmap roiBitmap = null;
        Scalar green = new Scalar(0, 255, 0, 255);
        Mat sourceMat = new Mat(1280, 720, CvType.CV_8UC3);
        sourceMat = input;

        Imgproc.blur(input, sourceMat, new Size(20,30));//50, 70

        Mat roiTmp = sourceMat.clone();
        //bitmapWidth = sourceBitmap.getWidth();
        Log.e("bitmapWidth", String.valueOf(1280));
        final Mat hsvMat = new Mat();
        sourceMat.copyTo(hsvMat);

        // convert mat to HSV format for Core.inRange()
        Imgproc.cvtColor(hsvMat, hsvMat, Imgproc.COLOR_RGB2HSV);


        Scalar lowerblack = new Scalar(205, 176, 0);         // lower color border for BLACK 100 100 100
        Scalar upperblack = new Scalar(240, 255, 150);      // upper color border for BLACK 180 255 255

        //^WORKING FOR RED+BLUE: 100, 80, 80 - - -  255, 225, 225
        //RED CONE ONLY: 150, 80, 80 - - - 255, 225, 225

        Scalar testRunL = new Scalar(70, 30, 0); // lower Green   83 100 51
        Scalar testRunU = new Scalar(255, 255, 255); // upper Green

        Core.inRange(hsvMat, lowerblack, upperblack, roiTmp);   // select only blue pixels
        // find contours
        List<MatOfPoint> contours = new ArrayList<>();
        List<RotatedRect> boundingRects = new ArrayList<>();
        Imgproc.findContours(roiTmp, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        // find appropriate bounding rectangles
        for (MatOfPoint contour : contours) {
            MatOfPoint2f areaPoints = new MatOfPoint2f(contour.toArray());
            RotatedRect boundingRect = Imgproc.minAreaRect(areaPoints);


            double rectangleArea = boundingRect.size.area();

            // test min ROI area in pixels
            if (rectangleArea > 3000 && rectangleArea < 800000) {//400000
                Point rotated_rect_points[] = new Point[4];
                boundingRect.points(rotated_rect_points);
                Rect rect3 = Imgproc.boundingRect(new MatOfPoint(rotated_rect_points));

                Log.e("blackArea", String.valueOf(rect3.area()));
                // test horizontal ROI orientation
                if (rect3.height > rect3.width) {
                    Imgproc.rectangle(sourceMat, rect3.tl(), rect3.br(), green, 3);
                    xBlack = rect3.br().x;
                    xBlackCenter = (rect3.br().x + rect3.tl().x) / 2;
                    yBlack = rect3.br().y;//bottom
                    battHeight = (rect3.br().y - rect3.tl().y); //batt height in pixel
                    Log.e("BLACKBR, TL", String.valueOf(rect3.br().y) + "," + String.valueOf(rect3.tl().y));
                }

            }

        }
        roiBitmap = Bitmap.createBitmap(sourceMat.cols(), sourceMat.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(sourceMat, roiBitmap);

        //return sourceMat;
        return roiTmp;


}

}



 */