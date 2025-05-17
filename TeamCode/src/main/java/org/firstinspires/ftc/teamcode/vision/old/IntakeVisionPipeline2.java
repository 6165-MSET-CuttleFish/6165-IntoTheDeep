package org.firstinspires.ftc.teamcode.vision.old;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.vision.Pipelines.Color;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class IntakeVisionPipeline2 extends OpenCvPipeline {
    private static final Rect ROI = new Rect(new Point(400, 325), new Point(700, 550));
    private static final Scalar RECT_COLOR = new Scalar(100, 100, 100);
    // HSV color ranges
    private static final Scalar RED_LOWER = new Scalar(0, 100, 100);
    private static final Scalar RED_UPPER = new Scalar(10, 255, 255);
    private static final Scalar BLUE_LOWER = new Scalar(100, 100, 100);
    private static final Scalar BLUE_UPPER = new Scalar(130, 255, 255);
    private static final Scalar YELLOW_LOWER = new Scalar(20, 100, 100);
    private static final Scalar YELLOW_UPPER = new Scalar(30, 255, 255);
    private final MultipleTelemetry telemetry;
    private final Mat mat = new Mat();
    private final Mat thresholdMat = new Mat();
    public Color detectedColor = Color.NONE;

    public IntakeVisionPipeline2(MultipleTelemetry t) {
        telemetry = t;
    }

    public Color getDetectedColor() {
        return detectedColor;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Mat roi = mat.submat(ROI);

        double redPercentage = getColorPercentage(roi, RED_LOWER, RED_UPPER);
        double bluePercentage = getColorPercentage(roi, BLUE_LOWER, BLUE_UPPER);
        double yellowPercentage = getColorPercentage(roi, YELLOW_LOWER, YELLOW_UPPER);

        if (redPercentage > bluePercentage && redPercentage > yellowPercentage && redPercentage > 10) {
            detectedColor = Color.RED;
        } else if (bluePercentage > redPercentage && bluePercentage > yellowPercentage && bluePercentage > 10) {
            detectedColor = Color.BLUE;
        } else if (yellowPercentage > redPercentage && yellowPercentage > bluePercentage && yellowPercentage > 10) {
            detectedColor = Color.NEUTRAL;
        } else {
            detectedColor = Color.NONE;
        }

        Imgproc.rectangle(input, ROI.tl(), ROI.br(), RECT_COLOR, 2);

        updateTelemetry(redPercentage, bluePercentage, yellowPercentage);

        return input;
    }

    private double getColorPercentage(Mat roi, Scalar lowerBound, Scalar upperBound) {
        Core.inRange(roi, lowerBound, upperBound, thresholdMat);
        return Core.countNonZero(thresholdMat) * 100.0 / ROI.area();
    }

    @SuppressLint("DefaultLocale")
    private void updateTelemetry(double redPercentage, double bluePercentage, double yellowPercentage) {
        telemetry.addData("Detected Color", detectedColor.name());
        telemetry.addData("Red Percentage", String.format("%.2f%%", redPercentage));
        telemetry.addData("Blue Percentage", String.format("%.2f%%", bluePercentage));
        telemetry.addData("Yellow Percentage", String.format("%.2f%%", yellowPercentage));
        telemetry.update();
    }
}