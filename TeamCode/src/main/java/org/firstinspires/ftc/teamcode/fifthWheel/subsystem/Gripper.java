package org.firstinspires.ftc.teamcode.fifthWheel.subsystem;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Gripper {
    // private Servo grip;
    public ServoEx flipLeft;
    public ServoEx flipRight;
    public ServoEx gripLeft;
    public ServoEx gripRight;

    // intake, ground, low, medium, high, up
    private static final double LEVELS[] = { -35, -20, -55, -90, 125 };
    private static final double OPEN = -20;
    private static final double CLOSE = -85;

    public Gripper(HardwareMap hwMap, String fl, String fr, String gl, String gr) {
        flipLeft = new SimpleServo(hwMap, fl, -200, 200);
        flipRight = new SimpleServo(hwMap, fr, -200, 200);
        gripLeft = new SimpleServo(hwMap, gl, -200, 200);
        gripRight = new SimpleServo(hwMap, gr, -200, 200);
        flipLeft.setInverted(true);
        gripLeft.setInverted(true);
    }

    public void setLevel(int i) {
        switch (i) {
            case -1: // up
                flipLeft.turnToAngle(LEVELS[4]);
                flipRight.turnToAngle(LEVELS[4]);
                break;
            default:
                flipLeft.turnToAngle(LEVELS[i]);
                flipRight.turnToAngle(LEVELS[i]);
                break;
        }
    }

    public void open() {
        gripLeft.turnToAngle(OPEN);
        gripRight.turnToAngle(OPEN);
    }

    public void close() {
        gripLeft.turnToAngle(CLOSE);
        gripRight.turnToAngle(CLOSE);
    }

    public void moveUp() {
        flipLeft.rotateByAngle(5);
        flipRight.rotateByAngle(5);
    }

    public void moveDown() {
        flipLeft.rotateByAngle(-5);
        flipRight.rotateByAngle(-5);
    }

    public void moveOpen() {
        gripLeft.rotateByAngle(5);
        gripRight.rotateByAngle(5);
    }

    public void moveClose() {
        gripLeft.rotateByAngle(-5);
        gripRight.rotateByAngle(-5);
    }
}
