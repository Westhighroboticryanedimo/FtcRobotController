package org.firstinspires.ftc.teamcode.slowBolon;

import android.graphics.Color;
import android.graphics.Color.*;
import android.os.Build;

import androidx.annotation.RequiresApi;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class duckyPipeline extends OpenCvPipeline
{
    Color pixelcolor;
    int tapecolor;
    int duckycolor;
    int whichspot;
    int width, height, tapeadecuado, duckyadecuado;
    int lefttapelimit, righttapelimit;
    int[] bytebuffer;
    int[] rgb;
    private Mat YCrCb = new Mat();
    private Mat Cb = new Mat();

    //testing
    public int leasttapediff = 255; public int leastduckydiff = 255;

    // colors
    public double greatestred;
    public double greatestyellow, greatestgreen;
    public int gx, gy;
    public int w;

    boolean tapepixel, duckypixel;
    @RequiresApi(api = Build.VERSION_CODES.O)
    @Override
    public Mat processFrame(Mat input) {
        return input;
    }
    /*public Mat processFrame(Mat input) {
        w = input.width();
        greatestred = 0; greatestyellow = 2000; greatestgreen = 0;
        //
        // Convert to YCrCb
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);

        // Extracts the Cb channel to the 'Cb' variable
        Core.extractChannel(YCrCb, Cb, 0);

        //int mean = (int) (Core.mean(Cb.submat(new Rect(300, 200, 1, 1))).val[0]);
        //int mean2 = (int) (Core.mean(Cb.submat(new Rect(300, 200, 1, 1))).val[2]);
        //leasttapediff = mean;
        //leastduckydiff = mean2;

        //check all pixels and return location of dook
        for(int y = 40; y < input.height()-40; y+=6) {
            for(int x = 0; x < input.width(); x+=6) {

                try{
                    Rect r = new Rect(x-10, y-10, 20, 20);
                    Scalar mean = Core.mean(input.submat(r));
                    if(
                            //Core.mean(input.submat(r)).val[0] >= greatestred
                            mean.val[0] >= greatestred
                            &&
                            //Core.mean(input.submat(r)).val[2] >= greatestgreen
                            mean.val[2] >= greatestgreen
                            &&
                            //Core.mean(input.submat(r)).val[1] <= 100
                            mean.val[1] <= 100
                    ) {
                        greatestred = mean.val[0]; greatestgreen = mean.val[2];
                        greatestyellow = (mean.val[0]+mean.val[2])/2;
                        //greatestyellow = (Core.mean(input.submat(r)).val[0] + Core.mean(input.submat(r)).val[2])/2;
                        //greatestyellow = (Core.mean(input.submat(r)).val[0]);
                        gx = x; gy = y;
                    }

                    //if( (Math.abs(Core.mean(YCrCb.submat(r)).val[2]))>=greatestred) {greatestred = Core.mean(YCrCb.submat(r)).val[2];}
                    //if(Math.abs(Core.mean(input.submat(r)).val[2])>=greatestred) {greatestred = Core.mean(input.submat(r)).val[2];}
                } catch(Exception e) {}
            }
        }
        Point REGION_RIGHT_A = new Point(gx,gy);
        Point REGION_RIGHT_B = new Point(gx+20,gy+20);

        Imgproc.rectangle(
                input, // Buffer to draw on
                REGION_RIGHT_A, // First point which defines the rectangle
                REGION_RIGHT_B, // Second point which defines the rectangle
                new Scalar(0, 0, 255), // The color the rectangle is drawn in
                6); // Thickness of the rectangle lines

        return input;

    }*/

    @RequiresApi(api = Build.VERSION_CODES.O)
    private double closeness(int aa, int bb) {
        Color a = Color.valueOf(aa);
        Color b = Color.valueOf(bb);
        return ((diff(a.red(),b.red()))/255+(diff(a.green(),b.green()))/255+(diff(a.blue(),b.blue()))/255)/3;
    }
    @RequiresApi(api = Build.VERSION_CODES.O)
    private double difference(Color a, Color b) {
        return (diff(a.red(),b.red())+diff(a.green(),b.green())+diff(a.blue(),b.blue()));
    }
    private double diff(double a, double b) {return Math.abs(a-b);}
    public int getspot() {return whichspot;}
}

// removed