package org.firstinspires.ftc.teamcode.thirdWheel.auto.actions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.thirdWheel.hardware.DriveThirdWheel;
import org.firstinspires.ftc.teamcode.thirdWheel.hardware.lift.Lift;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class Freight extends BaseHardware {

    private DriveThirdWheel drive;
    private Lift lift;
    int color = 1;

    public Freight(LinearOpMode opMode, DriveThirdWheel d, Lift l) {
        super(opMode);
        init(d, l);
    }

    public void init(DriveThirdWheel d, Lift l) {
        drive = d;
        lift = l;
    }

    public void setup(int c) {
        color = c;
    }

    public void move(int way) {
        lift.override(1, -1);
        lift.assist();
        while (!lift.arrived()) {
            lift.assist();
        }
        if (color == 1) {
            drive.frrNormie(40, 0, -1*way, 4, 0, -1, 0, 0, 0);
        } else if (color == 2) {
            drive.frrNormie(40, 0, 1*way, 4, 0, -1, 0, 0, 0);
        }
    }

    public void pick() {
        lift.override(0, 0);
        lift.assist();
        while (!lift.arrived()) {
            lift.assist();
        }
        if (color == 1) {
            drive.frrNormie(0, 0, 0, 0, 0, 0, 0.5, 0, -1);
        } else if (color == 2) {
            drive.frrNormie(0, 0, 0, 0, 0, 0, 0.5, 0, 1);
        }
        drive.move(0.25, 6, 0);
        lift.override(-1, 2);
        lift.assist();
        drive.move(0.25, 6, 180);
        if (color == 1) {
            drive.frrNormie(0, 0, 0, 0, 0, 0, 0.5, 0, 1);
        } else if (color == 2) {
            drive.frrNormie(0, 0, 0, 0, 0, 0, 0.5, 0, -1);
        }
    }

    public void place() {
        lift.override(3, -1);
        lift.assist();
        while (!lift.arrived()) {
            lift.assist();
        }
        drive.frrNormie(0, 0, 0, 14, 0, 1, 0, 0, 0);
        drive.stop();
        lift.override(-1, 1);
        for (int i = 0; i < 400; ++i) {
            lift.assist();
        }
        lift.override(1, 2);
        lift.assist();
        while (!lift.arrived()) {
            lift.assist();
        }
        drive.frrNormie(0, 0, 0, 14, 0, -1, 0, 0, 0);
        //drive.frrNormie(0, 0, 0, 16, 0, -1, 0.2,  0, dir);
    }
}
