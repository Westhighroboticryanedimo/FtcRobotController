package org.firstinspires.ftc.teamcode.thirdWheel.auto.actions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Gyro;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.thirdWheel.hardware.DriveThirdWheel;
import org.firstinspires.ftc.teamcode.thirdWheel.hardware.lift.Lift;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class Barcode extends BaseHardware {

    private DriveThirdWheel drive;
    private Lift lift;
    private DistanceSensor distanceSensor;
    private final int PIECE_DIST = 16;
    private int level = 1;
    private int dir = 1;
    public double d1 = 0;
    public double d2 = 0;

    public Barcode(LinearOpMode opMode, DriveThirdWheel d, Lift l, DistanceSensor ds) {
        super(opMode);
        init(d, l, ds);
    }

    public void init(DriveThirdWheel d, Lift l, DistanceSensor ds) {
        drive = d;
        lift = l;
        distanceSensor = ds;
    }

    private void setSide(int side) {
        if (side == 1) {
            dir = -1;
        } else if (side == 2) {
            dir = 1;
        }
    }

    public void setup(int side) {
        setSide(side);
        drive.move(0.5, 4, 0);
    }

    public boolean there() {
        return distanceSensor.getDistance(DistanceUnit.INCH) < PIECE_DIST;
    }

    public void detect() {
        d1 = dist();
        if (there()) {
            level = 1;
        } else {
            drive.turn(0.25, 15*dir);
            d2 = dist();
            if (there()) {
                level = 2;
            } else {
                level = 3;
            }
            drive.turn(0.25, -15*dir);
        }
        drive.stop();
        if (dir == -1) {
            level = -level + 4;
        }
    }

    public void put() {
        lift.override(level, -1);
        lift.assist();
        while (!lift.arrived()) {
            lift.assist();
        }
        //drive.frrNormie(0, 0, 0, 16, 0, 1, 0.2, 0.25, -dir);
        if (level != 3) {
            drive.frrNormie(18, 0, -dir, 18, 0, 1, 0, 0, 0);
        } else {
            drive.frrNormie(18, 0, -dir, 19, 0, 1, 0, 0, 0);
        }
        drive.stop();
        lift.override(-1, 1);
        for (int i = 0; i < 400; ++i) {
            lift.assist();
        }
        drive.frrNormie(0, 0, 0, 15, 0, -1, 0, 0, 0);
        lift.override(1, 2);
        lift.assist();
        while (!lift.arrived()) {
            lift.assist();
        }
        drive.stop();
        //drive.frrNormie(0, 0, 0, 16, 0, -1, 0.2,  0, dir);
    }

    public double dist() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }
}
