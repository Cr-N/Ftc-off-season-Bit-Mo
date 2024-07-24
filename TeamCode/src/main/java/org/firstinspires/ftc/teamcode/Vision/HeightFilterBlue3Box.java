package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class HeightFilterBlue3Box implements VisionProcessor {
    Telemetry telemetry;
    public Rect rectLeft = new Rect(0, 145, 174, 280);
    public Rect rectMiddle = new Rect(199, 174, 291, 216);
    public Rect rectRight = new Rect(497, 145, 142, 277);
    private static final int HEIGHT_THRESHOLD = 35; // Adjust this value as needed
    Selected selection = Selected.NONE;
    Mat submat = new Mat();
    Mat hsvMat = new Mat();
    Mat mask = new Mat();
    Rect largestBlobRect = new Rect();
    List<MatOfPoint> contours = new ArrayList<>();
    Mat hierarchy = new Mat();

    public HeightFilterBlue3Box(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration cameraCalibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Reset largestBlobRect for the new frame
        largestBlobRect = new Rect();

        double redBlobAreaLeft = getRedBlobArea(hsvMat, rectLeft);
        double redBlobAreaMiddle = getRedBlobArea(hsvMat, rectMiddle);
        double redBlobAreaRight = getRedBlobArea(hsvMat, rectRight);

        telemetry.addData("CASE: ", selection);
        telemetry.addData("Left box area: ", redBlobAreaLeft);
        telemetry.addData("Middle box area: ", redBlobAreaMiddle);
        telemetry.addData("Right box area: ", redBlobAreaRight);
        telemetry.addData("Largest Blob Rect: ", largestBlobRect.toString());
        telemetry.update();

        if ((redBlobAreaLeft > redBlobAreaMiddle) && (redBlobAreaLeft > redBlobAreaRight)) {
            selection = Selected.LEFT;
        } else if ((redBlobAreaMiddle > redBlobAreaLeft) && (redBlobAreaMiddle > redBlobAreaRight)) {
            selection = Selected.MIDDLE;
        } else {
            selection = Selected.RIGHT;
        }

        return selection;
    }

    protected double getRedBlobArea(Mat input, Rect rect) {
        submat = input.submat(rect);

        Scalar lowerRed = new Scalar(105, 60, 70); // Lower bound of HSV for blue // 110 60 80
        Scalar upperRed = new Scalar(140, 255, 255); // Upper bound of HSV for blue

        // Create a binary mask for the red color
        Core.inRange(submat, lowerRed, upperRed, mask);

        // Find contours in the binary mask
        contours.clear();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxBlobArea = 0;
        // Iterate through contours to find the largest blob area with height above threshold
        for (MatOfPoint contour : contours) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            if (boundingRect.height > HEIGHT_THRESHOLD) {
                double area = Imgproc.contourArea(contour);
                if (area > maxBlobArea) {
                    maxBlobArea = area;
                    // Update the largestBlobRect if the blob is in the current rect
                    Rect adjustedRect = new Rect(boundingRect.x + rect.x, boundingRect.y + rect.y, boundingRect.width, boundingRect.height);
                    largestBlobRect = adjustedRect;
                }
            }
        }

        return maxBlobArea;
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);
        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.GREEN);

        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);

        android.graphics.Rect drawDetectionBoundingBox = makeGraphicsRect(largestBlobRect, scaleBmpPxToCanvasPx);

        selection = (Selected) userContext;
        switch (selection) {
            case LEFT:
                canvas.drawRect(drawRectangleLeft, selectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
            case MIDDLE:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, selectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
            case RIGHT:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, selectedPaint);
                break;
            case NONE:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
        }

        // Draw the detection bounding box
        if (largestBlobRect != null && largestBlobRect.width > 0 && largestBlobRect.height > 0) {
            Paint boundingBoxPaint = new Paint();
            boundingBoxPaint.setColor(Color.BLUE);
            boundingBoxPaint.setStyle(Paint.Style.STROKE);
            boundingBoxPaint.setStrokeWidth(scaleCanvasDensity * 2);
            canvas.drawRect(drawDetectionBoundingBox, boundingBoxPaint);
        }
    }

    public Selected getSelection() {
        return selection;
    }

    public enum Selected {
        NONE,
        LEFT,
        MIDDLE,
        RIGHT
    }
}
