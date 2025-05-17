package org.firstinspires.ftc.teamcode.vision.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class IntakePipeline extends OpenCvPipeline {
    private Mat hsvMat = new Mat();
    private Mat yellowMask = new Mat();
    private Mat redMask = new Mat();
    private Mat blueMask = new Mat();
    private String dominantColor;
    private Color color;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        Scalar blueLowHSV = new Scalar(200, 10, 30);
        Scalar blueHighHSV = new Scalar(255, 70, 80);
        Scalar redLowHSV = new Scalar(340, 50, 30);
        Scalar redHighHSV = new Scalar(360, 90, 90);
        Scalar yellowLowHSV = new Scalar(0, 60, 55);
        Scalar yellowHighHSV = new Scalar(60, 90, 70);

        Core.inRange(hsvMat, blueLowHSV, blueHighHSV, blueMask);
        Core.inRange(hsvMat, redLowHSV, redHighHSV, redMask);
        Core.inRange(hsvMat, yellowLowHSV, yellowHighHSV, yellowMask);

        Rect rectMiddle = new Rect(new Point(0, 0), new Point(320, 240));
        Imgproc.rectangle(input, rectMiddle, new Scalar(64, 64, 64), 2);

        Mat middleYellow = yellowMask.submat(rectMiddle);
        Mat middleRed = redMask.submat(rectMiddle);
        Mat middleBlue = blueMask.submat(rectMiddle);

        dominantColor = detectDominantColor(middleYellow, middleRed, middleBlue);

        middleYellow.release();
        middleRed.release();
        middleBlue.release();

        return input;
    }

    public String getDominantColor() {
        return dominantColor;
    }

    public String detectDominantColor(Mat yellowMask, Mat redMask, Mat blueMask) {
        int yellowCount = Core.countNonZero(yellowMask);
        int redCount = Core.countNonZero(redMask);
        int blueCount = Core.countNonZero(blueMask);

        String result = "none";
        if (yellowCount > redCount && yellowCount > blueCount) {
            color = Color.NEUTRAL;
            result = "yellow";
        } else if (redCount > yellowCount && redCount > blueCount) {
            color = Color.RED;
            result = "red";
        } else if (blueCount > yellowCount && blueCount > redCount) {
            color = Color.BLUE;
            result = "blue";
        }

        return result + " BC: " + blueCount + " YC: " + yellowCount + " RC: " + redCount;
    }

    public Color getColor() {
        return color;
    }
}
