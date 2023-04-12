package org.firstinspires.ftc.teamcode.fifthWheel.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;

import java.lang.Math;
import java.util.ArrayList;

@Config
public class IntakeCam {

    public static OpenCvCamera camera;
    private SignalDeterminationPipeline signalPipeline;
    private ConeStackPipeline coneStackPipeline;
    public static int lowY = 0;
    public static int lowCr = 173;
    public static int lowCb = 40;
    public static int highY = 255;
    public static int highCr = 255;
    public static int highCb = 255;

    public static int redLowY = 0;
    public static int redLowCr = 170;
    public static int redLowCb = 80;
    public static int redHighY = 255;
    public static int redHighCr = 255;
    public static int redHighCb = 140;

    public static int blueLowY = 40;
    public static int blueLowCr = 70;
    public static int blueLowCb = 160;
    public static int blueHighY = 170;
    public static int blueHighCr = 130;
    public static int blueHighCb = 255;

    public IntakeCam(HardwareMap hwMap, boolean red) {
        setupOpenCV(hwMap, red);
    }

    private void setupOpenCV(HardwareMap hwMap, boolean red) {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        WebcamName camName = hwMap.get(WebcamName.class, "intakeCam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(camName, cameraMonitorViewId);
        signalPipeline = new SignalDeterminationPipeline();
        coneStackPipeline = new ConeStackPipeline(red);
        beginSignalDetection();
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
               // telemetry.addData("IntakeCam", "could not start streaming");
            }
        });
    }

    public void beginSignalDetection() {
        camera.setPipeline(signalPipeline);
    }

    public void beginConeStack() {
        camera.setPipeline(coneStackPipeline);
    }

    public int getPixelWidth() {
        return coneStackPipeline.getPixelWidth();
    }
    public int getMidWidth() {
        return coneStackPipeline.getMidWidth();
    }
    public double getYDistance() {
        return coneStackPipeline.getYDistance();
    }
    public double getXDistance(double heading) {
        return coneStackPipeline.getXDistance(heading);
    }

    public void changeRegionWidth(int w) {
        signalPipeline.REGION_WIDTH += w;
    }

    public void changeRegionHeight(int h) {
        signalPipeline.REGION_HEIGHT += h;
    }

    public void changeCornerX(int x) {
        signalPipeline.REGION_TOPLEFT.x += x;
    }

    public void changeCornerY(int y) {
        signalPipeline.REGION_TOPLEFT.y += y;
    }

    public int getRegionWidth() {
        return signalPipeline.REGION_WIDTH;
    }

    public int getRegionHeight() {
        return signalPipeline.REGION_HEIGHT;
    }

    public double getCornerX() {
        return signalPipeline.REGION_TOPLEFT.x;
    }

    public double getCornerY() {
        return signalPipeline.REGION_TOPLEFT.y;
    }

    public int getSignalFace() {
        return signalPipeline.getFace();
    }
    public int getAvgHue() {
        return signalPipeline.getAvgHue();
    }

    class SignalDeterminationPipeline extends OpenCvPipeline {
        private volatile int face = 1;

        Point REGION_TOPLEFT = new Point(415, 335);
        int REGION_WIDTH = 75;
        int REGION_HEIGHT = 115;
        final Scalar RED = new Scalar(255, 0, 0);

        Point region_pointA = new Point(REGION_TOPLEFT.x, REGION_TOPLEFT.y);
        Point region_pointB = new Point(REGION_TOPLEFT.x + REGION_WIDTH,
                                        REGION_TOPLEFT.y + REGION_HEIGHT);

        private volatile int avgHue = 0;
        // yellow, cyan, magenta
        int[] hues = new int[]{30, 90, 130};

        private Mat center;
        Mat hsv = new Mat();
        Mat hue = new Mat();

        private void inputToHSV(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(hsv, hue, 0);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToHSV(firstFrame);
            center = hue.submat(new Rect(region_pointA, region_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            region_pointA = new Point(REGION_TOPLEFT.x, REGION_TOPLEFT.y);
            region_pointB = new Point(REGION_TOPLEFT.x + REGION_WIDTH,
                    REGION_TOPLEFT.y + REGION_HEIGHT);
            center = hue.submat(new Rect(region_pointA, region_pointB));
            inputToHSV(input);
            Imgproc.rectangle(input, region_pointA, region_pointB, RED, 2);
            avgHue = (int) Core.mean(center).val[0];
            face = closestHue(avgHue);
            return input;
        }

        private int closestHue(int hue) {
            int closest = 1;
            int min = 361;
            for (int i = 0; i < 3; i++) {
                if (Math.abs(hue - hues[i]) < min) {
                    min = Math.abs(hue - hues[i]);
                    closest = i + 1;
                }
            }
            return closest;
        }

        public int getFace() {
            return face;
        }
        public int getAvgHue() {
            return avgHue;
        }
    }
    @Config
    public class ConeStackPipeline extends OpenCvPipeline {

        public ConeStackPipeline(boolean red) {
            if (red) {
                lowY = redLowY;
                lowCr = redLowCr;
                lowCb = redLowCb;
                highY = redHighY;
                highCr = redHighCr;
                highCb = redHighCb;
            } else {
                lowY = blueLowY;
                lowCr = blueLowCr;
                lowCb = blueLowCb;
                highY = blueHighY;
                highCr = blueHighCr;
                highCb = blueHighCb;
            }
        }

        Mat threshold = new Mat();
        Mat threshold1 = new Mat();
        Mat threshold2 = new Mat();
        Mat masked = new Mat();
        Mat yCrCb = new Mat();
        Mat filtered = new Mat();
        int biggestContourId = 0;
        double biggestContourArea = 0;
        Rect rect = new Rect();

        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));

        final double coneWidth = 4.0;
        // calibration pixels: 124
        // calibration distance: 20 3/4 in
        // object width: 4 in
        // 124*20.75/4 = 612.125
        final double focal = 643.25;
        // calibration pixels: 137
        // calibration distance: 20 3/4 in
        // object width: 4.6 in
        // 137*20.75/4.6 = ~618
        final double sideFocal = 618;
        final double fov = 54.5;
        private volatile int pixels = 0;
        private volatile int midWidth = 0;
        private double theta = 0;

        public int getPixelWidth() {
            return pixels;
        }
        public int getMidWidth() {
            return midWidth;
        }
        public double getYDistance() {
            return coneWidth*focal/pixels + 2.0;
        }
        public double getXDistance(double heading) {
            return Math.tan(Math.toRadians(theta)+(-heading))*getYDistance();
        }

        private void inputToYCrCb(Mat input) {
            Imgproc.cvtColor(input, yCrCb, Imgproc.COLOR_RGB2YCrCb);
        }

        @Override
        public void init(Mat firstFrame) { }

        @Override
        public Mat processFrame(Mat input) {
            biggestContourArea = 0.0;
            biggestContourArea = 0;
            yCrCb = new Mat();
            masked = new Mat();
            threshold = new Mat();
            filtered = new Mat();
            ArrayList<MatOfPoint> contoursList = new ArrayList<>();
            inputToYCrCb(input);
            Imgproc.erode(yCrCb, filtered, erodeElement);
            Imgproc.erode(filtered, filtered, erodeElement);

            Imgproc.dilate(filtered, filtered, dilateElement);
            Imgproc.dilate(filtered, filtered, dilateElement);
            // threshold for red cones
            // Core.inRange(hsv, new Scalar(lenientLowH, lenientLowS, lenientLowV),
            //         new Scalar(180, 255, 255), threshold1);
            // Core.inRange(hsv, new Scalar(0, lenientLowS, lenientLowV),
            //         new Scalar(lenientHighH, 255, 255), threshold2);
            int ly = lowY;
            int lcr = lowCr;
            int lcb = lowCb;
            int hy = highY;
            int hcr = highCr;
            int hcb = highCb;
            Core.inRange(filtered, new Scalar(ly, lcr, lcb), new Scalar(hy, hcr, hcb), threshold);
            // Core.bitwise_or(threshold1, threshold2, threshold);
            // color in the threshold
            Core.bitwise_and(filtered, filtered, masked, threshold);
            // find largest contour
            Imgproc.findContours(threshold, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            if (contoursList.size() > 0) {
                for (int i = 0; i < contoursList.size(); ++i) {
                    if (Imgproc.contourArea(contoursList.get(i)) > biggestContourArea) {
                        biggestContourArea = Imgproc.contourArea(contoursList.get(i));
                        biggestContourId = i;
                    }
                }
                rect = Imgproc.boundingRect(contoursList.get(biggestContourId));
                Imgproc.rectangle(masked, rect, new Scalar(128, 0, 255), 2);
                pixels = rect.width;
                midWidth = rect.x + rect.width/2 - 640/2;
                theta = ((rect.x + rect.width/2 - 640/2)/640.0)*fov;
            }

            Imgproc.cvtColor(masked, input, Imgproc.COLOR_YCrCb2RGB);
            yCrCb.release();
            masked.release();
            threshold.release();
            filtered.release();
            return input;
        }
    }
}
