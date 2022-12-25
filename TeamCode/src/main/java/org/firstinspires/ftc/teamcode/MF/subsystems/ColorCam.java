package org.firstinspires.ftc.teamcode.MF.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Scalar;
import org.opencv.core.Core;



public class ColorCam {
    private OpenCvCamera camera;
    private ColorPipeline colorPipeline = new ColorPipeline();

    public void cameraInit(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "ColorCam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.setPipeline(colorPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public int getColor() {
        return colorPipeline.getColor();
    }
    public int getHue() { return colorPipeline.getHueInt();}

    class ColorPipeline extends OpenCvPipeline {
//        0, 60, 120
        Mat hsv = new Mat();
        Mat hue = new Mat();
        Scalar hueScalar;
        int hueInt = 0;
        int color = 0;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(hsv, hue, 0);
            hueScalar = Core.mean(hue);
            hueInt = (int)Math.round(hueScalar.val[0]);
            if ((hueInt < 30) || (150 <= hueInt)) {
                color = 1;
            } else if (30 <= hueInt && hueInt < 90) {
                color = 2;
            } else if (90 <= hueInt && hueInt < 150) {
                color = 3;
            }

            return input;
        }

        public int getColor() {
            return(color);
        }
        public int getHueInt() {return(hueInt);}
    }
}
