package org.firstinspires.ftc.teamcode.ultimategoal.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.BaseHardware;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class Webcam extends BaseHardware {

    private OpenCvCamera webcam;
    private RingDeterminationPipeline pipeline;

    private static final int WIDTH = 800;
    private static final int HEIGHT = 600;

    public Webcam(OpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setupWebcam(hwMap);

    }

    public Webcam(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setupWebcam(hwMap);

    }

    private void setupWebcam(HardwareMap hwMap) {

        WebcamName webcamName = hwMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new RingDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPRIGHT));

    }

    @Override
    public void update() {

        if (isDebugMode) {

            opMode.telemetry.addData("Ring count: ", getRingStack());
            opMode.telemetry.addData("Position: ", getRingPosition());

        }

    }

    public int getRingStack() {

        return pipeline.getAnalysis();

    }

    public RingDeterminationPipeline.RingPosition getRingPosition() {

        return pipeline.getPosition();

    }

    private static class RingDeterminationPipeline extends OpenCvPipeline {

        private enum RingPosition {

            FOUR,
            ONE,
            NONE

        }

        private static final Scalar BLUE = new Scalar(0, 0, 255);
        private static final Scalar GREEN = new Scalar(0, 255, 0);

        private static final int REGION_WIDTH = 100;
        private static final int REGION_HEIGHT = 70;

        // The core values which define the location and size of the sample regions
        private static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(300 + REGION_WIDTH / 2,200 + REGION_HEIGHT / 2);

        private static final int FOUR_RING_THRESHOLD = 140;
        private static final int ONE_RING_THRESHOLD = 130;

        private Point region1_pointA = new Point(
                      REGION1_TOPLEFT_ANCHOR_POINT.x,
                      REGION1_TOPLEFT_ANCHOR_POINT.y);
        private Point region1_pointB = new Point(
                   REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                   REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        private Mat region1_Cb;
        private Mat YCrCb = new Mat();
        private Mat Cb = new Mat();
        private int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

         // This function takes the RGB frame, converts to YCrCb and extracts the Cb channel to the 'Cb' variable
        private void inputToCb(Mat input) {

            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);

        }

        @Override
        public void init(Mat firstFrame) {

            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));

        }

        @Override
        public Mat processFrame(Mat input) {

            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if (avg1 > FOUR_RING_THRESHOLD) position = RingPosition.FOUR;
            else if (avg1 > ONE_RING_THRESHOLD) position = RingPosition.ONE;
            else position = RingPosition.NONE;

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;

        }

        private int getAnalysis() { return avg1; }
        private RingPosition getPosition() { return position; }

    }

}
