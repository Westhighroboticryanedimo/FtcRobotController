package org.firstinspires.ftc.teamcode.fifthWheel.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

public class IntakeCam {

    private OpenCvCamera camera;
    private SignalDeterminationPipeline signalPipeline;

    public IntakeCam(HardwareMap hwMap) {
        setupOpenCV(hwMap);
    }
//     // Autonomous
//     public IntakeCam(LinearOpMode opMode, HardwareMap hwMap) {
// //        super(opMode);
//         setupOpenCV(hwMap);
//     }
//
//     // Teleop
//     public IntakeCam(OpMode opMode, HardwareMap hwMap) {
// //        super(opMode);
//         setupOpenCV(hwMap);
//     }

    private void setupOpenCV(HardwareMap hwMap) {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        WebcamName camName = hwMap.get(WebcamName.class, "intakeCam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(camName, cameraMonitorViewId);
        signalPipeline = new SignalDeterminationPipeline();
        camera.setPipeline(signalPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
//                telemetry.addData("IntakeCam", "could not start streaming");
            }
        });
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

        Point REGION_TOPLEFT = new Point(255, 210);
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
}
