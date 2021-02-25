package org.firstinspires.ftc.teamcode.freehug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
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

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.Constants.MILLIMETERS_PER_INCHES;

public class WebcamFree extends BaseHardware {

    // For switching mode
    private boolean isVuforia = false;

    // For OpenCV
    private OpenCvCamera webcam;
    private RingDeterminationPipeline pipeline;

    // For Vuforia
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final String VUFORIA_KEY = "Ad5o3uP/////AAABmZiHw0ExfEhJiGOj+a+euvV6cr7YdCbHAhFHDHW1effZUyKZ6QqwBvhDfgeu+flW5W6/uLkd5H2LYVeFlDHPdhbSwRx5mzj/IcQPSFKkhS6JkBM8DwnKa7B/c/jNykBPOXPG7RGpMRtqnq/A6jq5dZqqGgHGIkuN6TvED2ofqSsQD1F+QcBA9y8GHfenkdNlMIwzc+KAEtHwhFqXzG7K6PY9Yad/c/MV5U+aVZ0fcd9zKexVbj19mTJ7UwHCmkWKZvR7u3UHzF07/eWUO9KfqyGYtoYuX6hgfDw3/XTg5rLFMqvLpSCZK6jbLADHTkYMReDqu8wi4oPG4XlG8tCfLduGNLVXIm1NpYN/59Pg5ptq";
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    // Constants for perimeter targets
    private static final float HALF_FIELD = (float) (72 * MILLIMETERS_PER_INCHES);
    private static final float QUAD_FIELD = (float) (36 * MILLIMETERS_PER_INCHES);
    private static final float MM_TARGET_HEIGHT = (float) (6 * MILLIMETERS_PER_INCHES);

    // Camera position (inches)
    private static final float CAMERA_FORWARD_DISPLACEMENT = 4;
    private static final float CAMERA_VERTICAL_DISPLACEMENT = 0;
    private static final float CAMERA_LEFT_DISPLACEMENT = 0;

    // Used for Vuforia tracking
    private boolean isTargetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;
    private List<VuforiaTrackable> allTrackables = null;
    private String targetName = "";

    private enum Targets {

        BLUE_GOAL,
        RED_GOAL,
        RED_ALLIANCE,
        BLUE_ALLIANCE,
        FRONT_WALL

    }

    // Video resolution
    private static final int WIDTH = 800;
    private static final int HEIGHT = 600;

    // Get webcam name
    WebcamName webcamName;

    public WebcamFree(OpMode opMode, HardwareMap hwMap) {

        super(opMode);

        webcamName = hwMap.get(WebcamName.class, "webcam");

        //setupOpenCV(hwMap);
        setupVuforia(hwMap);

    }

    public WebcamFree(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode);

        webcamName = hwMap.get(WebcamName.class, "webcam");

        setupOpenCV(hwMap);

    }

    private void setupOpenCV(HardwareMap hwMap) {

        isVuforia = false;

        // Webcam setup
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new RingDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        // Start video streaming
        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPRIGHT));

    }

    private void setupVuforia(HardwareMap hwMap) {

        isVuforia = true;

        // Webcam setup
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // Set parameters
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;

        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects
        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName(Targets.BLUE_GOAL.name());
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName(Targets.RED_GOAL.name());
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName(Targets.RED_ALLIANCE.name());
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName(Targets.BLUE_ALLIANCE.name());
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName(Targets.FRONT_WALL.name());

        // Gather all trackables in list
        allTrackables = new ArrayList<>();
        allTrackables.addAll(targetsUltimateGoal);

        // Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -HALF_FIELD, MM_TARGET_HEIGHT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, HALF_FIELD, MM_TARGET_HEIGHT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-HALF_FIELD, 0, MM_TARGET_HEIGHT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(HALF_FIELD, QUAD_FIELD, MM_TARGET_HEIGHT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(HALF_FIELD, -QUAD_FIELD, MM_TARGET_HEIGHT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // Rotate the camera around its long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {

            phoneYRotate = -90;

        } else {

            phoneYRotate = 90;

        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {

            phoneXRotate = 90;

        }

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        for (VuforiaTrackable trackable : allTrackables) {

            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);

        }

        targetsUltimateGoal.activate();

    }

    public void update() {

        print("Is Vuforia: ", isVuforia);

         if (isVuforia) {

             // For Vuforia
             print("Tracking: ", getTrackingImage());
             print("X Displacement: ", getDisplacement()[0]);
             print("Y Displacement: ", getDisplacement()[1]);
             print("Z Displacement: ", getDisplacement()[2]);
             print("X: ", getTranslation()[0]);
             print("Y: ", getTranslation()[1]);
             print("Z: ", getTranslation()[2]);
             print("Roll: ", getOrientation()[0]);
             print("Pitch: ", getOrientation()[1]);
             print("Heading: ", getOrientation()[2]);

         } else {

             // For OpenCV
             print("Threshold: ", getInternalRingNum());
             print("Number of rings: ", getNumRings());

         }

        if (isVuforia) {

            // Check all the trackable targets to see which one (if any) is visible.
            isTargetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {

                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {

                    isTargetVisible = true;

                    // Set name
                    targetName = trackable.getName();

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {

                        lastLocation = robotLocationTransform;

                    }
                    break;

                }

            }

        }

    }

    public int getInternalRingNum() {

        return pipeline.getAnalysis();

    }

    public int getNumRings() {

        if (pipeline.getPosition() == RingDeterminationPipeline.RingPosition.NONE) return 0;
        else if (pipeline.getPosition() == RingDeterminationPipeline.RingPosition.ONE) return 1;
        else return 4;

    }

    public String getTrackingImage() {

        if (isTargetVisible) {

            return targetName;

        } else {

            return "No target detected";

        }

    }

    public float[] getTranslation() {

        if (isTargetVisible) {

            // Express position (translation) of robot in inches
            VectorF translation = lastLocation.getTranslation();

            // X,Y,Z
            return new float[]{translation.get(0) / (float) MILLIMETERS_PER_INCHES - CAMERA_FORWARD_DISPLACEMENT, translation.get(1) / (float) MILLIMETERS_PER_INCHES, translation.get(2) / (float) MILLIMETERS_PER_INCHES};

        } else {

            return new float[]{0, 0, 0};

        }

    }

    public float[] getOrientation() {

        if (isTargetVisible) {

            // Express the rotation of the robot in degrees
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);

            // Roll, pitch, heading
            return new float[]{rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle};

        } else {

            return new float[]{0, 0, 0};

        }

    }

    public float[] getDisplacement() {

        if (isTargetVisible) {

            int index;

            if (getTrackingImage().equals(Targets.BLUE_GOAL.name())) index = 0;
            else if (getTrackingImage().equals(Targets.RED_GOAL.name())) index = 1;
            else if (getTrackingImage().equals(Targets.BLUE_ALLIANCE.name())) index = 2;
            else if (getTrackingImage().equals(Targets.RED_ALLIANCE.name())) index = 3;
            else index = 4;

            // Get location
            VectorF imageLocation = allTrackables.get(index).getLocation().getTranslation();

            return new float[]{imageLocation.get(0) / (float) MILLIMETERS_PER_INCHES - getTranslation()[0], imageLocation.get(1) / (float) MILLIMETERS_PER_INCHES - getTranslation()[1], imageLocation.get(2) / (float) MILLIMETERS_PER_INCHES - getTranslation()[2]};

        } else {

            return new float[]{0, 0, 0};

        }

    }

    public void switchToVuforia(HardwareMap hwMap) {

        // Stop OpenCV streaming
        webcam.stopStreaming();
        webcam.closeCameraDevice();

        // Start Vuforia
        setupVuforia(hwMap);

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
        private static final int REGION_HEIGHT = 80;
        private static final int X_DIFF = -355;
        private static final int Y_DIFF = -90;

        // The core values which define the location and size of the sample regions
        private static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point((int) (WIDTH / 2 + REGION_WIDTH / 2 + X_DIFF), (int) (HEIGHT / 2 + REGION_HEIGHT / 2 + Y_DIFF));

        private static final int FOUR_RING_THRESHOLD = 165;
        private static final int ONE_RING_THRESHOLD = 140;

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
