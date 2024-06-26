package org.firstinspires.ftc.teamcode.Vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

// This is from the -Learn Java For FTC book by Alan G. Smith, released January 2,2024
// It will draw 3 boxes and color in red which one has an object based on the saturation( which one has more color)
// Be careful, white = Max Red + Max Green + Max Blue on the visual spectrum so this might not work if you are trying to detect a colorful object(maybe make the object white?) but if you can ensure a black or grey background, it should work(maybe play with exposure - untested, don't know if that will work though -) maybe would be good if you have low lighting conditions and play matches in the dark :)
// intended to be used with EOCV-Sim (it works on V3.5.2) on a 320 by 240 camera resolution
// Made 26 06 2024
public class Average3BoxDetection implements VisionProcessor {
    public Rect rectLeft = new Rect(60, 104, 55, 55);
    public Rect rectMiddle = new Rect(140, 104, 55, 55);
    public Rect rectRight = new Rect(220, 104, 55, 55);
    Selected selection = Selected.NONE;
    Telemetry telemetry;
    Mat submat = new Mat();

    public Average3BoxDetection(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration cameraCalibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        double redRectLeft = getAvgRed(frame, rectLeft);
        double redRectMiddle = getAvgRed(frame, rectMiddle);
        double redRectRight = getAvgRed(frame, rectRight);

        if (telemetry != null) {
            if ((redRectLeft > redRectMiddle) && (redRectLeft > redRectRight)) {
                telemetry.addLine("LEFT");
                telemetry.update();
                return Selected.LEFT;
            } else if ((redRectMiddle > redRectLeft) && (redRectMiddle > redRectRight)) {
                telemetry.addLine("MIDDLE");
                telemetry.update();
                return Selected.MIDDLE;
            }
            telemetry.addLine("RIGHT");
            telemetry.update();
            return Selected.RIGHT;
        } else {
            // Handle the case when telemetry is null
            System.err.println("Telemetry is not initialized");
            return Selected.NONE;
        }
    }

    protected double getAvgRed(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[2]; // Red channel is at index 2 in BGR color space
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