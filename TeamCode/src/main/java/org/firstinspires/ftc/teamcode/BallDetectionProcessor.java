package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class BallDetectionProcessor implements VisionProcessor {

    private Mat hsvMat = new Mat();
    private Mat mask = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();

    // Define the color range for the ball (e.g., for a red ball)
    // These values will likely need to be tuned.
    public Scalar lower = new Scalar(0, 100, 100);
    public Scalar upper = new Scalar(10, 255, 255);

    private double largestContourArea = 0;
    private Point largestContourCenter = new Point();
    private double largestContourRadius = 0;
    private double largestContourCircularity = 0;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Not needed for this processor
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert the frame to HSV color space
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Create a mask to only keep pixels within the color range
        Core.inRange(hsvMat, lower, upper, mask);

        // Apply morphological operations to reduce noise
        Imgproc.erode(mask, mask, new Mat(), new Point(-1, -1), 2);
        Imgproc.dilate(mask, mask, new Mat(), new Point(-1, -1), 2);

        // Find contours
        contours.clear();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest contour
        largestContourArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > largestContourArea) {
                largestContourArea = area;
                largestContour = contour;
            }
        }

        if (largestContour != null) {
            // Calculate center and radius using circle fitting
            Point center = new Point();
            float[] radius = new float[1];
            Imgproc.minEnclosingCircle(largestContour, center, radius);
            largestContourCenter = center;
            largestContourRadius = radius[0];

            // Calculate circularity
            double perimeter = Imgproc.arcLength(largestContour, true);
            largestContourCircularity = 4 * Math.PI * largestContourArea / (perimeter * perimeter);
        } else {
            largestContourCenter.x = 0;
            largestContourCenter.y = 0;
            largestContourRadius = 0;
            largestContourCircularity = 0;
        }

        return frame; // Return the original frame
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (largestContourArea > 0) {
            Paint paint = new Paint();
            paint.setColor(Color.YELLOW);
            paint.setStyle(Paint.Style.STROKE);
            paint.setStrokeWidth(4);

            float centerX = (float) largestContourCenter.x * scaleBmpPxToCanvasPx;
            float centerY = (float) largestContourCenter.y * scaleBmpPxToCanvasPx;
            float radius = (float) largestContourRadius * scaleBmpPxToCanvasPx;

            canvas.drawCircle(centerX, centerY, radius, paint);
        }
    }

    public Point getLargestContourCenter() {
        return largestContourCenter;
    }

    public double getLargestContourArea() {
        return largestContourArea;
    }
    
    public double getLargestContourRadius() {
        return largestContourRadius;
    }
    
    public double getLargestContourCircularity() {
        return largestContourCircularity;
    }
}
