package org.firstinspires.ftc.teamcode.vision.old;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class IntakeVisionPipeline3 extends OpenCvPipeline {

    private String detectedColor = "blue";

    @Override
    public Mat processFrame(Mat input) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_BGR2HSV);

        Mat yellowMask = new Mat();
        Mat redMask = new Mat();
        Mat blueMask = new Mat();

        Scalar lowerYellow = new Scalar(20, 100, 100);
        Scalar upperYellow = new Scalar(30, 255, 255);
        Scalar lowerRed1 = new Scalar(0, 100, 100);
        Scalar upperRed1 = new Scalar(10, 255, 255);
        Scalar lowerRed2 = new Scalar(160, 100, 100);
        Scalar upperRed2 = new Scalar(180, 255, 255);
        Scalar lowerBlue = new Scalar(100, 150, 0);
        Scalar upperBlue = new Scalar(140, 255, 255);

        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);
        Core.inRange(hsvFrame, lowerRed1, upperRed1, redMask);
        Mat redMask2 = new Mat();
        Core.inRange(hsvFrame, lowerRed2, upperRed2, redMask2);
        Core.add(redMask, redMask2, redMask);
        Core.inRange(hsvFrame, lowerBlue, upperBlue, blueMask);

        detectedColor = detectDominantColor(yellowMask, redMask, blueMask);

        return input;
    }

    private String detectDominantColor(Mat yellowMask, Mat redMask, Mat blueMask) {
        int yellowCount = Core.countNonZero(yellowMask);
        int redCount = Core.countNonZero(redMask);
        int blueCount = Core.countNonZero(blueMask);

        if (yellowCount > redCount && yellowCount > blueCount) {
            return "yellow";
        } else if (redCount > yellowCount && redCount > blueCount) {
            return "red";
        } else if (blueCount > yellowCount && blueCount > redCount) {
            return "blue";
        } else {
            return "null";
        }
    }

    public String getDetectedColor() {
        return detectedColor;
    }
}

