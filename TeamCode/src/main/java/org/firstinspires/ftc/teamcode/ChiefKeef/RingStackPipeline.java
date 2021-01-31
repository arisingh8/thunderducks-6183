package org.firstinspires.ftc.teamcode.ChiefKeef;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RingStackPipeline extends OpenCvPipeline {

    private int threshold = 50;

    private double aspectRatio;
    private int minWidth = 100;

    private double maxVal = 0;
    private int maxValIdx = 0;
    private Rect maxRect = new Rect();

    private int ringState;

    private Mat hierarchy = new Mat();
    private Mat mask = new Mat();
    private Mat result = new Mat();

    private int horizon = (100 / 320) * 720;

    @Override
    public Mat processFrame(Mat input)
    {
        Scalar color = new Scalar(255, 255, 255);

        Mat hierarchy = new Mat();
        Mat mask = new Mat();
        Mat result = new Mat();

        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2YCrCb);

        // Threshold of blue in HSV space
        Scalar lower = new Scalar(0, 140, 40);
        Scalar upper = new Scalar(255.0, 230.0, 95.0);

        Core.inRange(input, lower, upper, mask);
        Core.bitwise_and(input, input, result, mask);

        Imgproc.cvtColor(result, result, Imgproc.COLOR_YCrCb2BGR);
        Imgproc.cvtColor(result, result, Imgproc.COLOR_BGR2GRAY);
        Imgproc.blur(result, result, new Size(3, 3));

        Imgproc.Canny(result, result, threshold, threshold * 3);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(result, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        result = Mat.zeros(result.size(), CvType.CV_8UC3);
        Imgproc.drawContours(result, contours, -1, color, 2, Imgproc.LINE_8, hierarchy, 0, new Point());

        for (int i = 0; i < contours.size(); i++) {
            Rect contourArea = Imgproc.boundingRect(contours.get(i));
            if (contourArea.y + contourArea.height > horizon) {
                if (maxVal < contourArea.area()) {
                    maxVal = contourArea.area();
                    maxRect = contourArea;
                    maxValIdx = i;
                }
            }
        }

        aspectRatio = Double.valueOf(maxRect.width)/Double.valueOf(maxRect.height);

        Imgproc.putText(result, String.valueOf(aspectRatio), new Point(maxRect.x + maxRect.width, maxRect.y + maxRect.height), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, color);
        Imgproc.drawContours(result, contours, maxValIdx, new Scalar(0, 255, 0), 2, Imgproc.LINE_8, hierarchy, 0, new Point());
        Imgproc.rectangle(result, maxRect, color);

        return input;
    }

    public int getRingStack() {
        if (maxRect.width < minWidth) {
            ringState = 0;
            return ringState;
        }
        if (aspectRatio > 2) {
            ringState = 1;
            return ringState;
        } else {
            ringState = 4;
            return ringState;
        }
    }
}