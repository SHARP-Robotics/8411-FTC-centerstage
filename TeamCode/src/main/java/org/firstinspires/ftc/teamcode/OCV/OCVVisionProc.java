package org.firstinspires.ftc.teamcode.OCV;

import android.bluetooth.le.ScanResult;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Color;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class OCVVisionProc implements VisionProcessor {
    public Rect rectLeft = new Rect(0, 260, 180, 120);
    public Rect rectMiddle = new Rect(200, 260, 230, 120);
    public Rect rectRight = new Rect(440, 260, 200, 120);
    Selected selection = Selected.NONE;

    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration){
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos){
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        double satRectLeft = getAvgSaturation(hsvMat, rectLeft);
        double satRectMiddle = getAvgSaturation(hsvMat, rectMiddle);
        double satRectRight = getAvgSaturation(hsvMat, rectRight);

        if ((satRectLeft > satRectMiddle) && (satRectLeft > satRectRight)) {
            return Selected.LEFT;
        } else if ((satRectMiddle > satRectRight) && (satRectMiddle > satRectLeft)) {
            return Selected.MIDDLE;
        } else if((satRectRight > satRectMiddle) && (satRectRight > satRectLeft)){
            return Selected.RIGHT;
        }
        return Selected.NONE;
    }

    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx){
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonselectedPaint = new Paint(selectedPaint);
        nonselectedPaint.setColor(Color.GREEN);

        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft,scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle,scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);

        selection = (Selected) userContext;
        switch (selection){
            case LEFT:
                canvas.drawRect(drawRectangleLeft, selectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonselectedPaint);
                canvas.drawRect(drawRectangleRight, nonselectedPaint);
                break;
            case MIDDLE:
                canvas.drawRect(drawRectangleLeft, nonselectedPaint);
                canvas.drawRect(drawRectangleMiddle, selectedPaint);
                canvas.drawRect(drawRectangleRight, nonselectedPaint);
                break;
            case RIGHT:
                canvas.drawRect(drawRectangleLeft, nonselectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonselectedPaint);
                canvas.drawRect(drawRectangleRight, selectedPaint);
                break;
            case NONE:
                canvas.drawRect(drawRectangleLeft, nonselectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonselectedPaint);
                canvas.drawRect(drawRectangleRight, nonselectedPaint);
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