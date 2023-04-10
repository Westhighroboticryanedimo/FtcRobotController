package org.firstinspires.ftc.teamcode.Wingman.subsystems;

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
import org.opencv.core.Point;
import org.opencv.core.Rect;



public class ColorCam {
    private OpenCvCamera camera;
    private ColorPipeline colorPipeline = new ColorPipeline();

    int p1X = 215;
    int p1Y = 315;
    int p2X = 245;
    int p2Y = 405;
    int sleeveType = 1;
// 0 = red green blue 1 = yellow cyan magenta

    public void change_p1X(int change) {
        p1X = p1X + change;
    }

    public void change_p2X(int change) {
        p2X = p2X + change;
    }

    public void change_p1Y(int change) {
        p1Y = p1Y + change;
    }

    public void change_p2Y(int change) {
        p2Y = p2Y + change;
    }

    public void set_p1X(int set) {p1X = set;}

    public void set_p2X(int set) {p2X = set;}

    public void set_p1Y(int set) {p1Y = set;}

    public void set_p2Y(int set) {p2Y = set;}

    public void set_sleeveType (int set) {sleeveType = set;}

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
    public int getp1X() {return colorPipeline.getp1X();}
    public int getp2X() {return colorPipeline.getp2X();}
    public int getp1Y() {return colorPipeline.getp1Y();}
    public int getp2Y() {return colorPipeline.getp2Y();}

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
            Mat focus = hsv.submat(new Rect(new Point(p1X, p1Y), new Point(p2X, p2Y))); //**********
            Core.extractChannel(focus, hue, 0); //Change "focus" to hsv
            hueScalar = Core.mean(hue);
            hueInt = (int)Math.round(hueScalar.val[0]);
            Imgproc.rectangle(input, new Point(p1X, p1Y), new Point(p2X, p2Y), new Scalar(255, 0, 0), 2); //************
            if (sleeveType == 0) {
                if ((hueInt < 30) || (150 <= hueInt)) {
                    color = 1;
                } else if (30 <= hueInt && hueInt < 90) {
                    color = 2;
                } else if (90 <= hueInt && hueInt < 150) {
                    color = 3;
                }
            } else if (sleeveType == 1) {
                if ((hueInt < 60) && (0 <= hueInt)) {
                    color = 1;
                } else if (60 <= hueInt && hueInt < 120) {
                    color = 2;
                } else if (120 <= hueInt && hueInt < 180) {
                    color = 3;
                }
            }

            return input;
        }

        public int getColor() {
            return(color);
        }
        public int getHueInt() {return(hueInt);}
        public int getp1X() {return(p1X);}
        public int getp2X() {return(p2X);}
        public int getp1Y() {return(p1Y);}
        public int getp2Y() {return(p2Y);}
    }
}
