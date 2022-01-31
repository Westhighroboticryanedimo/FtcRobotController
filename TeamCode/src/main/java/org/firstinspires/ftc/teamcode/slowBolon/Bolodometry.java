package org.firstinspires.ftc.teamcode.slowBolon;

public class Bolodometry {
    double x,y, angle;
    double last_x, last_y, last_angle;
    double fl, fr, bl, br;
    double lfl, lfr, lbl, lbr;
    public void resetposition() {
        x = 0; y = 0; angle = 0;
    }
    public void updateposition(double infr, double infl, double inbr, double inbl, double inangle) {

        fl = infl; fr = infr; bl = inbl; br = inbr;

        double forward = (fr-lfr) + (fl-lfl) + (br-lbr) + (bl-lbl);
        y += forward;

        last_x = x; last_y = y; last_angle = angle;
        lbr = br; lbl = bl;
        lfr = fr; lfl = fl;
    }

    public double diff(double a, double b) {
        return Math.abs(a-b);
    }
}
