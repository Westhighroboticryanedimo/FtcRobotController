package org.firstinspires.ftc.teamcode.SlowBolon;

import android.graphics.Color;
import android.graphics.Color.*;
import android.os.Build;

import androidx.annotation.RequiresApi;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

class duckyPipeline extends OpenCvPipeline
{
    Color pixelcolor;
    int tapecolor, duckycolor;
    int whichspot;
    int width, height, tapeadecuado, duckyadecuado;
    int lefttapelimit, righttapelimit;
    int[] bytebuffer;
    int[] rgb;

    //testing
    public int leasttapediff = 255; public int leastduckydiff = 255;

    boolean tapepixel, duckypixel;
    @RequiresApi(api = Build.VERSION_CODES.O)
    @Override
    public Mat processFrame(Mat input)
    {
        lefttapelimit = width/2; righttapelimit = lefttapelimit;
        tapecolor = Color.RED; duckycolor = Color.YELLOW;
        //define rgb requirements to be considered tape
        tapeadecuado = 60;
        //and ducky
        duckyadecuado = 60;

        bytebuffer = new int[] {};
        whichspot = -1;
        rgb = new int[] {};

        width = input.width(); height = input.height();
        for(int y = 0; y < height; y++) {
            for(int x = 0; x < width; x++) {
                //for each pixel
                int pixel = input.get(x,y,bytebuffer); // hopefully hex value
                rgb[0] = Color.red(pixel);
                rgb[1] = Color.green(pixel);
                rgb[2] = Color.blue(pixel);


                if(closeness(pixel, duckycolor) <= duckyadecuado) {
                    duckypixel = true;
                } else {duckypixel = false;}
                if(closeness(pixel, tapecolor) <= tapeadecuado) {
                    tapepixel= true;
                    if(x > righttapelimit) {righttapelimit = x;}
                    else if(x < lefttapelimit) {lefttapelimit = x;}
                } else {tapepixel = false;}

                // doock
                if(duckypixel) {
                    whichspot = (int) ((3*(x-lefttapelimit)/(righttapelimit-lefttapelimit)));
                }

                // testing
                if(diff(pixel,tapecolor) < leasttapediff) {leasttapediff = (int)(diff(pixel,tapecolor));}
                if(diff(pixel,duckycolor) < leastduckydiff) {leastduckydiff = (int)(diff(pixel,duckycolor));}
            }
        }
        return input;
    }
    @RequiresApi(api = Build.VERSION_CODES.O)
    private double closeness(int aa, int bb) {
        Color a = Color.valueOf(aa);
        Color b = Color.valueOf(bb);
        return ((diff(a.red(),b.red()))/255+(diff(a.green(),b.green()))/255+(diff(a.blue(),b.blue()))/255)/3;
    }
    private double diff(float a, float b) {return Math.abs(a-b);}
    public int getspot() {return whichspot;}
}