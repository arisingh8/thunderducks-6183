package org.firstinspires.ftc.teamcode.ChiefKeef;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ShooterTargetingPipeline extends OpenCvPipeline {

    private int threshold = 50;
    private double peri;

    private MatOfPoint approxconv = new MatOfPoint();
    private MatOfPoint2f approx = new MatOfPoint2f();

    private MatOfPoint2f cnt = new MatOfPoint2f();

    private Scalar color = new Scalar(255, 255, 255);

    private Mat hierarchy = new Mat();
    private Mat mask = new Mat();
    private Mat result = new Mat();

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
        // Threshold of blue in HSV space
        Scalar lower = new Scalar(0, 100, 100);
        Scalar upper = new Scalar(10, 255, 255);

        Core.inRange(input, lower, upper, mask);
        Core.bitwise_and(input, input, result, mask);

        Imgproc.cvtColor(result, result, Imgproc.COLOR_HSV2RGB);
        Imgproc.cvtColor(result, result, Imgproc.COLOR_RGB2GRAY);
        Imgproc.blur(result, result, new Size(3, 3));

        Imgproc.Canny(result, result, threshold, threshold * 3);
        List<MatOfPoint> contours = new ArrayList<>();
        List<MatOfPoint> approxList = new ArrayList<>();
        Imgproc.findContours(result, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        result = Mat.zeros(result.size(), CvType.CV_8UC3);
        for (int i = 0; i < contours.size(); i++) {
            contours.get(i).convertTo(cnt, CvType.CV_32F);
            peri = Imgproc.arcLength(cnt, true);
            Imgproc.approxPolyDP(cnt, approx, 0.02* peri, true);

            Point[] points = approx.toArray();
            if (points.length == 8) {
                approx.convertTo(approxconv, CvType.CV_32S);
                approxList.add(approxconv);
            }
        }

        if (!approxList.isEmpty()) {
            double maxVal = 0;
            int maxValIdx = 0;
            for (int i = 0; i < approxList.size(); i++) {
                double contourArea = Imgproc.contourArea(approxList.get(i));
                if (maxVal < contourArea) {
                    maxVal = contourArea;
                    maxValIdx = i;
                }
                for(Point p: approxList.get(maxValIdx).toList()) {
                    String string = p.x+" "+p.y;
                    Imgproc.putText(result, string, p, Imgproc.FONT_HERSHEY_COMPLEX, 0.5, color);
                }
                Imgproc.drawContours(result, approxList, maxValIdx, color, 2, Imgproc.LINE_8, hierarchy, 0, new Point());
            }
        } else {
            for (int i = 0; i < contours.size(); i++) {
                Imgproc.drawContours(result, contours, i, color, 2, Imgproc.LINE_8, hierarchy, 0, new Point());
            }
        }
        return result;
    }
}