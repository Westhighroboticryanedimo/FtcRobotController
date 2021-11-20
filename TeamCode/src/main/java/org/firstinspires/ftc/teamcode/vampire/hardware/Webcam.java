package org.firstinspires.ftc.teamcode.vampire.hardware;

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

    // For OpenCV
    private OpenCvCamera webcam;
    private CargoDeterminationPipeline pipeline;

    // Video resolution
    private static final int WIDTH = 800;
    private static final int HEIGHT = 600;

    // Get webcam name
    WebcamName webcamName;

    public Webcam(OpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setupOpenCV(hwMap);

    }

    public Webcam(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setupOpenCV(hwMap);

    }

    private void setupOpenCV(HardwareMap hwMap) {

        // Webcam setup
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new CargoDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        // Start video streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() { webcam.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPRIGHT); }

            @Override
            public void onError(int errorCode) {}

        });

    }

    public void update() {

        // For OpenCV
        print("Threshold Left: ", pipeline.avgL);
        print("Threshold Mid: ", pipeline.avgM);
        print("Threshold Right: ", pipeline.avgR);
        print("Position: ", getCargoPos());

    }

    public char getCargoPos() {

        if (pipeline.getPosition() == CargoDeterminationPipeline.CargoPosition.BOTTOM)
            return 'b';
        else if (pipeline.getPosition() == CargoDeterminationPipeline.CargoPosition.MIDDLE)
            return 'm';
        else if (pipeline.getPosition() == CargoDeterminationPipeline.CargoPosition.TOP)
            return 't';
        else return ' ';

    }

    public int[] getRaw() {

        return pipeline.getRaw();

    }

    private static class CargoDeterminationPipeline extends OpenCvPipeline {

        private enum CargoPosition {

            BOTTOM,
            MIDDLE,
            TOP

        }

        private static final int REGION_WIDTH = 50;
        private static final int REGION_HEIGHT = 50;
        private static final Point REGION_LEFT = new Point(-100, -60);
        private static final Point REGION_MID = new Point(0, -60);
        private static final Point REGION_RIGHT = new Point(100, -60);

        // The core values which define the location and size of the sample regions
        private static final Point REGION_LEFT_TOPLEFT = new Point((int) (WIDTH / 2 + REGION_WIDTH / 2 + REGION_LEFT.x), (int) (HEIGHT / 2 + REGION_HEIGHT / 2 + REGION_LEFT.y));
        private static final Point REGION_MID_TOPLEFT = new Point((int) (WIDTH / 2 + REGION_WIDTH / 2 + REGION_MID.x), (int) (HEIGHT / 2 + REGION_HEIGHT / 2 + REGION_MID.y));
        private static final Point REGION_RIGHT_TOPLEFT = new Point((int) (WIDTH / 2 + REGION_WIDTH / 2 + REGION_RIGHT.x), (int) (HEIGHT / 2 + REGION_HEIGHT / 2 + REGION_RIGHT.y));

        // Regions
        private final Point REGION_LEFT_A = new Point(
                      REGION_LEFT_TOPLEFT.x,
                      REGION_LEFT_TOPLEFT.y);
        private final Point REGION_LEFT_B = new Point(
                   REGION_LEFT_TOPLEFT.x + REGION_WIDTH,
                   REGION_LEFT_TOPLEFT.y + REGION_HEIGHT);
        private final Point REGION_MID_A = new Point(
                REGION_MID_TOPLEFT.x,
                REGION_MID_TOPLEFT.y);
        private final Point REGION_MID_B = new Point(
                REGION_MID_TOPLEFT.x + REGION_WIDTH,
                REGION_MID_TOPLEFT.y + REGION_HEIGHT);
        private final Point REGION_RIGHT_A = new Point(
                REGION_RIGHT_TOPLEFT.x,
                REGION_RIGHT_TOPLEFT.y);
        private final Point REGION_RIGHT_B = new Point(
                REGION_RIGHT_TOPLEFT.x + REGION_WIDTH,
                REGION_RIGHT_TOPLEFT.y + REGION_HEIGHT);

        private Mat regionLeftCb;
        private Mat regionMidCb;
        private Mat regionRightCb;
        private Mat YCrCb = new Mat();
        private Mat Cb = new Mat();
        private int avgL;
        private int avgM;
        private int avgR;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile CargoPosition position;

        private void inputToCb(Mat input) {

            // Convert to YCrCb
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);

            // Extracts the Cb channel to the 'Cb' variable
            Core.extractChannel(YCrCb, Cb, 1);

        }

        @Override
        public void init(Mat firstFrame) {

            inputToCb(firstFrame);
            regionLeftCb = Cb.submat(new Rect(REGION_LEFT_A, REGION_LEFT_B));
            regionMidCb = Cb.submat(new Rect(REGION_MID_A, REGION_MID_B));
            regionRightCb = Cb.submat(new Rect(REGION_RIGHT_A, REGION_RIGHT_B));

        }

        @Override
        public Mat processFrame(Mat input) {

            inputToCb(input);

            // The average cb value in the region
            avgL = (int) Core.mean(regionLeftCb).val[0];
            avgM = (int) Core.mean(regionMidCb).val[0];
            avgR = (int) Core.mean(regionRightCb).val[0];

            // Draw rectangles
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    REGION_LEFT_A, // First point which defines the rectangle
                    REGION_LEFT_B, // Second point which defines the rectangle
                    new Scalar(0, 0, 255), // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    REGION_LEFT_A, // First point which defines the rectangle
                    REGION_LEFT_B, // Second point which defines the rectangle
                    new Scalar(0, 255, 0), // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
            // Draw rectangles
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    REGION_MID_A, // First point which defines the rectangle
                    REGION_MID_B, // Second point which defines the rectangle
                    new Scalar(0, 0, 255), // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    REGION_MID_A, // First point which defines the rectangle
                    REGION_MID_B, // Second point which defines the rectangle
                    new Scalar(0, 255, 0), // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill// Draw rectangles
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    REGION_RIGHT_A, // First point which defines the rectangle
                    REGION_RIGHT_B, // Second point which defines the rectangle
                    new Scalar(0, 0, 255), // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    REGION_RIGHT_A, // First point which defines the rectangle
                    REGION_RIGHT_B, // Second point which defines the rectangle
                    new Scalar(0, 255, 0), // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            // Get position
            int max = Math.max(avgL, Math.max(avgM, avgR));
            if (max == avgL) position = CargoPosition.BOTTOM;
            else if (max == avgM) position = CargoPosition.MIDDLE;
            else position = CargoPosition.TOP;

            return input;

        }

        private int[] getRaw() {

            return new int[] { avgL, avgM, avgR };

        }

        private CargoPosition getPosition() { return position; }

    }

}
