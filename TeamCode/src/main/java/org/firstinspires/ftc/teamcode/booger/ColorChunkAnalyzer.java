package org.firstinspires.ftc.teamcode.booger;

/*
import java.awt.image.BufferedImage;
import java.awt.Graphics2D;
import java.awt.Color;
*/
import java.util.*;
//import java.awt.image.*;
//import java.awt.geom.AffineTransform;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Build;

public class ColorChunkAnalyzer {
    
    //public BufferedImage toanalyze;
    public Bitmap toanalyze;

    public ArrayList<double[]> offlimits;
    private ArrayList<double[]> chunkpoints; // stores the points of the current chunk
    public ArrayList<ChunkData> chunks;
    public int pointsput;
    //private Graphics2D g;
    private int chunk;

    // calibration and navigation:
    public ChunkData calibrate; // data from [the right things in the right place]

    public void calibrate() { // set the current image to the calibration for red.
        calibrate = mainChunk();
    }

    public double[] getOffset() { // return as a vector the offset between calibrate and the current main chunk [x, y, and size]
        ChunkData m = mainChunk();
        double[] r = new double[] {m.middlex - calibrate.middlex, m.middley - calibrate.middley, (m.width*m.height) - (calibrate.width*calibrate.height)};
        if(Math.abs(r[0]) < 20) {
            r[0] = 0;
        }
        if(Math.abs(r[1]) < 20) {
            r[1] = 0;
        }
        return r;
        // [x offset, y offset, area offset]
    }

    //public void setImage(BufferedImage in) {
    public void setImageStep1(Bitmap in) {
        //toanalyze = new BufferedImage(in.getWidth()/3,in.getHeight()/3,BufferedImage.TYPE_INT_ARGB);
        toanalyze = Bitmap.createBitmap(in.getWidth(),in.getHeight(),Bitmap.Config.ARGB_8888);
        toanalyze = Bitmap.createBitmap(in);

        //g = toanalyze.createGraphics();
        for(int x = 0; x < toanalyze.getWidth(); x++) { //
            for(int y = 0; y < toanalyze.getHeight(); y++) { //
                //toanalyze.setPixel(x,y,in.getPixel(x*3,y*3));
                float[] hsb = new float[3];
                int point = colorAtPoint(x,y);
                //Color.RGBtoHSB(point.getRed(), point.getGreen(), point.getBlue(), hsb);
                Color.colorToHSV(point,hsb);
                hsb[1] = (float)roundToNearest(hsb[1],0.3); if(hsb[1] > 1) {hsb[1] = 1;}
                hsb[0] = (float)roundToNearest(hsb[0],0.06); if(hsb[0] > 1) {hsb[0] = 1;}
            }
        }
        offlimits = new ArrayList<double[]>();
        chunkpoints = new ArrayList<double[]>();
        chunk = 5;
        chunks = new ArrayList<ChunkData>();
    }
    public void setImageStep2() {
        for(int x = 0; x < toanalyze.getWidth(); x++) { // FIX PLEASE
            for(int y = 0; y < toanalyze.getHeight(); y++) { // FIX PLEASE
                float[] hsb = new float[3];
                int point = colorAtPoint(x,y);
                // since red is really annoying, we shall turn everything red into green. [red exists on both extremes of the hsv hue continuum]
                // red: > 0.93333 [pink red] or < 0.6666 [orange red] --> to 0.277777 [green]
                if(hsb[0] < 0.06666666 || hsb[0] > 0.9333333) {hsb[0] = (float)0.27777777;}

                // also, make the greens red.
                else if(hsb[0] < 0.4555555 && hsb[0] > 0.2166666) {hsb[0] = (float)0.9999999;}
            }
        }
    }

    public ArrayList<ChunkData> getColorChunks(int c) {
        ArrayList<ChunkData> result = new ArrayList<ChunkData>();
        // scan screen until pixel of desired color is reached
        for(int x = 1; x < toanalyze.getWidth(); x+=4) {
            for(int y = 1; y < toanalyze.getHeight(); y+=4) {
                //p(offlimits.size()+"");
                //System.out.println(x + " , " + y + " , w: " + toanalyze.getWidth());
                // when reached, recursively map the whole chunk of similarly colored pixels
                ArrayList<double[]> l = new ArrayList<double[]>();
                if(colorMatches(colorAtPoint(x,y),c,0.15) && !pointInList(new double[] {x,y},offlimits)) { // if this point is the right color and is not off limits
                    chunkpoints = new ArrayList<double[]>();
                    l = chunkPoints(l, c, x, y, /* next number is max steps */ 100); // *********************************** this
                    //if(colorMatches(colorAtPoint(x,y),red,0.1)) {g.setColor(Color.GREEN); g.fillRect(x,y,1,1);}
                    l = chunkpoints;
                    //p(l.size() + " is the size of the current chunk [points]");
                }
                // we now have access to the points that make up the currently scanning chunk of color. Now we need to record this chunk's data
                // create chunk color differences
                chunk += 10; if(chunk > 255) {chunk = 0;}
                if(l.size() > 0) {
                    System.out.println("adding chunk at " + x + " , " + y + " ... this many chunks: " + chunks.size());
                    // [width, height, middle point position], and add it to a list of chunks.
                    ChunkData data = new ChunkData();
                    data.setDataFromPoints(l);
                    chunks.add(data);
                    //if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {drawRect((int)data.minx,(int)data.miny,(int)data.width,(int)data.height,Color.valueOf(Color.RED));}
                    break;
                }
                
                // continue until all areas have been recorded on the list of chunks and return
            }
        }
        if(chunks.size()>0) {ChunkData largest = mainChunk();}
        //if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {drawRect((int)largest.minx,(int)largest.miny,(int)largest.width,(int)largest.height, Color.valueOf(Color.RED));}
        p(chunks.size() + " chunks exist");
        return result;
    }

    public void drawRect(int x, int y, int w, int h, Color c) {
        for(int px = x; px < x + w; px++) {
            for(int py = y; py < y + h; py++) {
                if(px == x || py == y || px == x + w || py == py + h) {
                    toanalyze.setPixel(x,y,Color.parseColor(c.toString()));
                }
            }
        }
    }

    public boolean colorMatches(int a, int b, double diff) {
        //System.out.println("red: " + a.getRed() + " h: " + Color.RGBtoHSB(a.getRed(), a.getGreen(), a.getBlue(), null)[0]);
        float[] ahsb = new float[3];
        float[] bhsb = new float[3];
        //Color.RGBtoHSB(a.getRed(), a.getGreen(), a.getBlue(), ahsb);
        //Color.RGBtoHSB(b.getRed(), b.getGreen(), b.getBlue(), bhsb);
        Color.colorToHSV(a,ahsb);
        Color.colorToHSV(b,bhsb);
        return ((Math.abs(ahsb[0]-bhsb[0]) < diff) && (Math.abs(ahsb[1]-bhsb[1]) < 0.3));
        //return Math.abs(Color.RGBtoHSB(a.getRed(), a.getGreen(), a.getBlue(), null)[0] - Color.RGBtoHSB(b.getRed(), b.getGreen(), b.getBlue(), null)[0]) < diff;
    }

    public int colorAtPoint(int x, int y) {
        int col = toanalyze.getPixel(x,y);
        return col;
    }

    public double roundToNearest(double toround, double threshold) {
        return Math.ceil(toround/threshold)*threshold;
    }

    public boolean pointInList(double[] a, ArrayList<double[]> b) { // is the point in the list?
        boolean in = false;
        for(int i = 0; i < b.size(); i++) {
            //if(a.equals(b.get(i))) {in = true; break;}
            if(a[0] == b.get(i)[0] && a[1] == b.get(i)[1]) {in = true; break;}
        }
        return in;
    }

    public boolean outOfBounds(int x, int y) { // is this point outside of the image
        return (x <= 0 || x >= toanalyze.getWidth()-1 || y <= 0 || y >= toanalyze.getHeight()-1);
    }

    // below: recursion
    public ArrayList<double[]> chunkPoints(ArrayList<double[]> list, int c, int x, int y, int step) {
        // parameters: colorchunk points, points that have already been claimed, the color to look for, the x and y of thr current operating point
        if(!outOfBounds(x,y) && step > 0) {
            if(pointInList(new double[] {x,y}, offlimits)) { // if this point is off limits for any reason, stop.
                //p("in bounds, yet [color wrong or offlimits] " + x + " , " + y);
                return list;
            } else if(colorMatches(colorAtPoint(x,y),c,0.15)) { // else spread to nearby points
                offlimits.add(new double[] {x,y});
                chunkpoints.add(new double[] {x,y});
                pointsput++;
                //g.setColor(new Color(chunk,chunk,chunk));
                //g.fillRect(x,y,1,1);
                
                chunkPoints(list, c, x+2, y, step-1);
                chunkPoints(list, c, x-2, y, step-1);
                chunkPoints(list, c, x, y-2, step-1);
                return chunkPoints(list, c, x, y+2, step-1);
            } else {return list;}
        } else {
            //p("out of bounds " + x + " , " + y);
            return list;
        }
        
    }

    public ChunkData mainChunk() { // return the data of the main [largest] chunk
        int biggestindex = 0;
        ChunkData biggest = chunks.get(biggestindex);
        for(int i = 0; i < chunks.size(); i++) {
            ChunkData ichunk = chunks.get(i);
            double a = ichunk.width*ichunk.height;
            biggest = chunks.get(biggestindex);
            double ba = biggest.width*biggest.height;
            if(a > ba) {
                biggestindex = i;
                biggest = ichunk;
            }
        }
        return biggest;
    }

    public void p(String in) {System.out.println(in);}

    //public BufferedImage returnImage() {return toanalyze;}
    public Bitmap returnImage() {return toanalyze;}

}
