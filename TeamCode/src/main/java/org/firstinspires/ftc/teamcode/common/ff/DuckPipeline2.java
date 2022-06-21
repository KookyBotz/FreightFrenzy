package org.firstinspires.ftc.teamcode.common.ff;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


import static org.firstinspires.ftc.teamcode.common.ff.DuckPipeline2.BarcodeConstants.*;

public class DuckPipeline2 extends OpenCvPipeline {
    public static class BarcodeConstants {
        public static boolean DISPLAY = true;
        public static Scalar DISPLAY_COLOR = new Scalar(255, 0, 0);
        public static Scalar LOWER_LIMIT = new Scalar(0.0, 150.0, 120.0);
        public static Scalar UPPER_LIMIT = new Scalar(255.0, 255.0, 255.0);
        public static int BORDER_LEFT_X = 0;   //amount of pixels from the left side of the cam to skip
        public static int BORDER_RIGHT_X = 0;   //amount of pixels from the right of the cam to skip
        public static int BORDER_TOP_Y = 78;   //amount of pixels from the top of the cam to skip
        public static int BORDER_BOTTOM_Y = 0;   //amount of pixels from the bottom of the cam to skip

        //y is fot the outpiut

        public static int VARIANCE = 50;
        public static double MIN_AREA = 100;


    }

    public Exception debug;


    private int loopcounter = 0;
    private int ploopcounter = 0;

    private final Mat mat = new Mat();
    private final Mat processed = new Mat();

    private Rect maxRect = new Rect();

    private double maxArea = 0;
    private boolean first = false;

    public Telemetry telemetry;

    public DuckPipeline2(Telemetry t) {
        telemetry = t;
    }

    public DuckPipeline2() {
    }

    @Override
    public Mat processFrame(Mat input) {
        try {
            // Process Image
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(mat, processed, 2);
            Imgproc.threshold(processed, processed, 100, 110, 0);

            //Core.inRange(mat, LOWER_LIMIT, UPPER_LIMIT, processed);
            // Core.bitwise_and(input, input, output, processed);

            // Remove Noise
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());
            // GaussianBlur
            Imgproc.GaussianBlur(processed, processed, new Size(5.0, 15.0), 0.00);
            // Find Contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            // Draw Contours
            if (DISPLAY) Imgproc.drawContours(input, contours, -1, DISPLAY_COLOR);

            // Loop Through Contours
            for (MatOfPoint contour : contours) {
                Point[] contourArray = contour.toArray();

                // Bound Rectangle if Contour is Large Enough
                if (contourArray.length >= 15) {
                    MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                    Rect rect = Imgproc.boundingRect(areaPoints);

                    // if rectangle is larger than previous cycle or if rectangle is not larger than previous 6 cycles > then replace
                    if ((rect.area() > maxArea || loopcounter - ploopcounter > 6)
                            && rect.x > BORDER_LEFT_X && rect.x + rect.width < input.width() - BORDER_RIGHT_X
                            && rect.y > BORDER_TOP_Y && rect.y + rect.height < input.height() - BORDER_BOTTOM_Y
                    ) {
                        maxArea = rect.area();
                        maxRect = rect;
                        ploopcounter++;
                        loopcounter = ploopcounter;
                        first = true;
                    }
                    areaPoints.release();
                }
                contour.release();
            }
            mat.release();
            processed.release();
            if (contours.isEmpty()) {
                maxRect = new Rect();
            }
            if (first && maxRect.area() > MIN_AREA) {
                if (DISPLAY) Imgproc.rectangle(input, maxRect, DISPLAY_COLOR, 2);
            }
            // Draw Borders
            if (DISPLAY) {
                Imgproc.rectangle(input, new Rect(BORDER_LEFT_X, BORDER_TOP_Y, input.width() - BORDER_RIGHT_X - BORDER_LEFT_X, input.height() - BORDER_BOTTOM_Y - BORDER_TOP_Y), DISPLAY_COLOR, 2);

                // Display Data

                Imgproc.putText(input, "Area: " + getRectArea() + " Midpoint: " + getRectMidpointXY().x + " , " + getRectMidpointXY().y + " Selection: " + getDuckie(), new Point(20, input.height() - 20), Imgproc.FONT_HERSHEY_PLAIN, 0.6, DISPLAY_COLOR, 1);
            }
            loopcounter++;
        } catch (Exception e) {
            debug = e;
            boolean error = true;
        }
        if (telemetry != null) {
            telemetry.addLine(getDuckie() + "");
            telemetry.update();
        }

        try {
            Thread.sleep(100);
        } catch (InterruptedException ignored) {
        }

        return input;
    }

    public int getRectHeight() {
        return maxRect.height;
    }

    public int getRectWidth() {
        return maxRect.width;
    }

    public int getRectX() {
        return maxRect.x;
    }

    public int getRectY() {
        return maxRect.y;
    }

    public double getRectMidpointX() {
        return getRectX() + (getRectWidth() / 2.0);
    }

    public double getRectMidpointY() {
        return getRectY() + (getRectHeight() / 2.0);
    }

    public Point getRectMidpointXY() {
        return new Point(getRectMidpointX(), getRectMidpointY());
    }

    public double getRectArea() {
        return maxRect.area();
    }

    public double getDuckie() {
        System.out.println(getRectMidpointX());
        return getRectMidpointX();
    }
}