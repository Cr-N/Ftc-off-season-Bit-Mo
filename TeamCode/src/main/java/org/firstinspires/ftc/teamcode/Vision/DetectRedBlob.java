//// DETECT RED BLOB OF MAXIMUM AREA ON SCREEN BETWEEN lower and higher HSV

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
public class DetectRedBlob implements VisionProcessor {
    Telemetry telemetry;
    Mat hsvMat = new Mat();
    Mat mask = new Mat();
    List<MatOfPoint> contours = new ArrayList<>();
    Mat hierarchy = new Mat();
    Rect largestBlobRect = new Rect();

    public DetectRedBlob(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration cameraCalibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        Scalar lowerRed = new Scalar(135, 60, 80); // Lower bound of HSV(Hue,Saturation,Value) for red
        Scalar upperRed = new Scalar(180, 255, 255); // Upper bound of HSV(Hue,Saturation,Value) for red

        // Create a binary mask for the red color
        Core.inRange(hsvMat, lowerRed, upperRed, mask);

        // Find contours in the binary mask
        contours.clear();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxBlobArea = 0;
        largestBlobRect = new Rect();

        // Iterate through contours to find the largest blob and its bounding rectangle
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxBlobArea) {
                maxBlobArea = area;
                largestBlobRect = Imgproc.boundingRect(contour);
            }
        }

        // Log details for debugging
        telemetry.addData("Contours Count: ", contours.size());
        telemetry.addData("Max Blob Area: ", maxBlobArea);
        telemetry.addData("Largest Blob Rect: ", largestBlobRect.toString());
        telemetry.update();

        return largestBlobRect;
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
        Paint paint = new Paint();
        paint.setColor(Color.RED);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(scaleCanvasDensity * 4);

        android.graphics.Rect drawRectangle = makeGraphicsRect(largestBlobRect, scaleBmpPxToCanvasPx);
        canvas.drawRect(drawRectangle, paint);
    }
}