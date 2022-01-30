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
    private final int PIECE_DIST = 24;
    private int level = 1;

    public Barcode(LinearOpMode opMode, DriveThirdWheel d, Lift l, DistanceSensor ds) {
        super(opMode);
        init(d, l, ds);
    }

    public void init(DriveThirdWheel d, Lift l, DistanceSensor ds) {
        drive = d;
        lift = l;
        distanceSensor = ds;
    }

    public void setup() {
        drive.move(0.25, 4, 0);
    }

    public void detect(int side) {
        for (; level <= 2; ++level) {
            if (distanceSensor.getDistance(DistanceUnit.INCH) < PIECE_DIST) {
                break;
            }
            if (side == 2) {
                drive.turn(0.25, 15);
            } else if (side == 1) {
                drive.turn(0.25, -15);
            }
        }
        drive.stop();
        if (side == 1) {
            level = -level + 4;
        }
        if (side == 2) {
            for (int i = 1; i < level; ++i) {
                drive.turn(0.25, -15);
            }
        } else if (side == 1) {
            for (int i = 1; i < (-level + 4); ++i) {
                drive.turn(0.25, 15);
            }
        }
    }

    public void put(int side) {
        if (side == 1) {
            drive.move(0.25, 25, 90);
        } else if (side == 2) {
            drive.move(0.25, 25, -90);
        }
        lift.override(level, -1);
        lift.assist();
        while (!lift.arrived()) {
            lift.assist();
        }
        drive.move(0.25, 28, 0);
        lift.override(-1, 1);
        for (int i = 0; i < 400; ++i) {
            lift.assist();
        }
        lift.override(-1, 2);
        lift.assist();
        drive.move(0.25, 36, 180);
        lift.override(0, -1);
        lift.assist();
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < 3)  {
            lift.assist();
        }
    }

    public void park(int color, int side) {
        lift.override(2, -1);
        lift.assist();
        while (!lift.arrived()) {
            lift.assist();
        }
        if (color == 1) {
            /*if (side == 1) {
                drive.move(0.5, 60, 270);
            } else if (side == 2) {
                drive.move(0.5, 120, 270);
            }*/
            drive.move(0.5, 80, 265);
            drive.move(0.5, 15, 0);
        } else if (color == 2) {
            /*if (side == 1) {
                drive.move(0.25, 120, 90);
            } else if (side == 2) {
                drive.move(0.5, 60, 90);
            }*/
            drive.move(0.5, 80, 95);
            drive.move(0.5, 15, 0);
        }
        lift.override(0, -1);
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < 3)  {
            lift.assist();
        }
    }

    public double dist() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }
}
