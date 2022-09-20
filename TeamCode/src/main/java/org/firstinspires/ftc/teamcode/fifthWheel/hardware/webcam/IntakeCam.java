package org.firstinspires.ftc.teamcode.fifthWheel.hardware.webcam;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;
// import org.openftc.easyopencv.OpenCvCameraFactory;
// import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;

import java.lang.Math;

public class IntakeCam {

    private OpenCvCamera camera;
    private SignalDeterminationPipeline signalPipeline;

    // Autonomous
    public IntakeCam(LinearOpMode opMode, HardwareMap hwMap) {
        super(opMode, hwMap);
        setupOpenCV(hwMap);
    }

    // Teleop
    public IntakeCam(OpMode opMode, HardwareMap hwMap) {
        super(opMode, hwMap);
        setupOpenCV(hwMap);
    }

    private void setupOpenCV(HardwareMap hwMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName camName = hardwareMap.get(WebcamName.class, "intakeCam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(camName, cameraMonitorViewId);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                signalPipeline = new SignalDeterminationPipeline();
                camera.setPipeline(signalPipeline);
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("IntakeCam", "could not start streaming");
            }
        });
    }

    public int getSignalFace() {
        return signalPipeline.getFace();
    }

    class SignalDeterminationPipeline extends OpenCvPipeline {
        private volatile int face = 1;

        static final Point REGION_TOPLEFT = new Point(140, 100);
        static final int REGION_WIDTH = 40;
        static final int REGION_HEIGHT = 40;

        Point region_pointA = new Point(REGION_TOPLEFT.x, REGION_TOPLEFT.y);
        Point region_pointB = new Point(REGION_TOPLEFT.x + REGION_WIDTH,
                                        REGION_TOPLEFT.y + REGION_HEIGHT);

        int avgHue = 0;
        // yellow, cyan, magenta
        int[] hues = new int[]{60, 180, 300};

        private Mat center;

        private void inputToHSV(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToHSV(firstFrame);
            center = firstFrame.submat(new Rect(region_pointA, region_pointB);
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToHSV(input);
            Imgproc.rectangle( input, region_pointA, region_pointB, RED, 2);
            avgHue = (int) Core.mean(center).val[0];
            face = closestHue(avgHue);
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
    }
}
