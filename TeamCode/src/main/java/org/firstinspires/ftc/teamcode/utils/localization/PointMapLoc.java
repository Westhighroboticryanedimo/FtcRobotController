package org.firstinspires.ftc.teamcode.utils.localization;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;
import java.lang.Math;

final class Point {
    double x = 0;
    double y = 0;
    double theta = 0;
    Pose2d poseAtCapture = new Pose2d();
}

enum State {
    SEARCH, HORIZ, VERT, DONE
}

public class PointMapLoc extends BaseHardware {
    public Drive drive;
    private DistanceSensor f;
    private DistanceSensor l;
    private DistanceSensor b;
    private DistanceSensor r;
    private DcMotor rotator;

    private ArrayList<Point> pointMap = new ArrayList<Point>();
    int mapSize = 0;

    boolean dir = true;
    private double ticksPerRev = 100;
    private double quarterTicks = ticksPerRev /4;
    private double distFromCenter = 2;

    public PointMapLoc(LinearOpMode opMode, HardwareMap hwMap, Drive d) {
        super(opMode);
        init(hwMap, d);
    }

    public PointMapLoc(OpMode opMode, HardwareMap hwMap, Drive d) {
        super(opMode);
        init(hwMap, d);
    }

    private void init(HardwareMap hwMap, Drive d) {
        drive = d;
        f = hwMap.get(DistanceSensor.class, "f");
        r = hwMap.get(DistanceSensor.class, "r");
        b = hwMap.get(DistanceSensor.class, "b");
        l = hwMap.get(DistanceSensor.class, "l");
        rotator = hwMap.get(DcMotor.class, "rotator");
    }

    public void scan() {
        ArrayList<Point> q1 = new ArrayList<Point>();
        ArrayList<Point> q2 = new ArrayList<Point>();
        ArrayList<Point> q3 = new ArrayList<Point>();
        ArrayList<Point> q4 = new ArrayList<Point>();
        double d1 = 0;
        double d2 = 0;
        double d3 = 0;
        double d4 = 0;
        Point point = new Point();
        double theta = 0;
        Pose2d poseAtCapture = new Pose2d();
        double target = 0;

        if (dir) {
            rotator.setPower(1);
            target = quarterTicks;
        } else {
            rotator.setPower(-1);
            target = 0;
        }
        dir = !dir;
        while (Math.abs(target - rotator.getCurrentPosition()) < quarterTicks) {
            d1 = f.getDistance(DistanceUnit.INCH);
            d2 = r.getDistance(DistanceUnit.INCH);
            d3 = b.getDistance(DistanceUnit.INCH);
            d4 = l.getDistance(DistanceUnit.INCH);
            theta = drive.getExternalHeading() + ticksToRadians(rotator.getCurrentPosition());
            poseAtCapture = drive.getPoseEstimate();

            // TODO: add rolling shutter undistortion
            // roadrunner-style coords are used
            point.x = d1 * Math.cos(theta);
            point.y = d1 * Math.sin(theta);
            point.theta = theta;
            point.poseAtCapture = poseAtCapture;
            q1.add(point);
            point.x = d2 * Math.cos(theta + Math.PI/2);
            point.y = d2 * Math.sin(theta + Math.PI/2);
            point.theta = theta;
            point.poseAtCapture = poseAtCapture;
            q2.add(point);
            point.x = d3 * Math.cos(theta + Math.PI);
            point.y = d3 * Math.sin(theta + Math.PI);
            point.theta = theta;
            point.poseAtCapture = poseAtCapture;
            q3.add(point);
            point.x = d4 * Math.cos(theta + 3*Math.PI/2);
            point.y = d4 * Math.sin(theta + 3*Math.PI/2);
            point.theta = theta;
            point.poseAtCapture = poseAtCapture;
            q4.add(point);
        }
        rotator.setPower(0);
        pointMap.addAll(q1);
        pointMap.addAll(q2);
        pointMap.addAll(q3);
        pointMap.addAll(q4);
        mapSize = pointMap.size();
    }

    public void correctDists() {
        for (int i = 0; i < pointMap.size(); ++i) {
            pointMap.get(i).x = distFromCenter*Math.cos(pointMap.get(i).theta);
            pointMap.get(i).y = distFromCenter*Math.sin(pointMap.get(i).theta);
        }
    }

    public void undistort() {
        for (int i = 0; i < mapSize; ++i) {
            pointMap.get(i).x = pointMap.get(i).x - (pointMap.get(pointMap.size()-1).poseAtCapture.getX() - pointMap.get(i).x);
            pointMap.get(i).y = pointMap.get(i).y - (pointMap.get(pointMap.size()-1).poseAtCapture.getY() - pointMap.get(i).y);
        }
    }

    // TODO: use an actual filter instead of rounding, probably kalman or something
    public void filter() {
        for (int i = 0; i < mapSize; ++i) {
            pointMap.get(i).x = Math.round(pointMap.get(i).x);
            pointMap.get(i).y = Math.round(pointMap.get(i).y);
        }
    }

    public void process() {
        correctDists();
        undistort();
        filter();
    }

    public Vector2d getPos() {
        double x = 0;
        double y = 0;
        State state = State.SEARCH;
        int i = 1;
        boolean wall = true;
        while (state != State.DONE) {
            switch (state) {
                case SEARCH:
                    if (Math.abs(pointMap.get(i).x - pointMap.get(i-1).x) < 0.5) {
                        state = State.HORIZ;
                        i += 1;
                        break;
                    }
                    if (Math.abs(pointMap.get(i).y - pointMap.get(i-1).y) < 0.5) {
                        state = State.VERT;
                        i += 1;
                        break;
                    }
                    i += 1;
                    break;
                case HORIZ:
                    wall = true;
                    for (int count = 2; count < 4; ++count) {
                        if (!(Math.abs(pointMap.get(i).x - pointMap.get(i-1).x) < 0.5)) {
                            wall = false;
                            count = 0;
                            break;
                        }
                        i += 1;
                    }
                    if (wall) {
                        x = pointMap.get(i).x;
                    }
                    break;
                case VERT:
                    wall = true;
                    for (int count = 2; count < 4; ++count) {
                        if (!(Math.abs(pointMap.get(i).y - pointMap.get(i-1).y) < 0.5)) {
                            wall = false;
                            count = 0;
                            break;
                        }
                        i += 1;
                    }
                    if (wall) {
                        y = pointMap.get(i).y;
                    }
                    break;
            }
        }
        return new Vector2d(x, y);
    }

    public Vector2d update() {
        scan();
        process();
        return getPos();
    }

    private double ticksToRadians(double ticks) {
        return 2*Math.PI/ticksPerRev;
    }
}
