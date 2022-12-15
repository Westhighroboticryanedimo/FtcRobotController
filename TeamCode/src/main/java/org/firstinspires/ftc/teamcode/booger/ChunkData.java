package org.firstinspires.ftc.teamcode.booger;

import java.util.*;

public class ChunkData {
    public double width, height, middlex, middley;
    public double[] topleft, bottomright; // coordinatesdouble minx = pp[0];
    double minx;
    double maxx;
    double miny;
    double maxy;

    public void setDataFromPoints(ArrayList<double[]> p) {
        int num = p.size();
        double[] pp = p.get(0);
        minx = pp[0];
        maxx = pp[0];
        miny = pp[1];
        maxy = pp[1];
        for(int i = 0; i < num; i+=10) {
            pp = p.get(i);
            if(pp[0] < minx) {minx = pp[0];}
            if(pp[0] > maxx) {maxx = pp[0];}
            if(pp[1] < miny) {miny = pp[1];}
            if(pp[1] > maxy) {maxy = pp[1];}
        }

        topleft = new double[] {minx,miny};
        bottomright = new double[] {maxx,maxy};

        width = maxx - minx;
        height = maxy - miny;

        middlex = maxx - (width/2);
        middley = maxy - (height/2);
    }
}
