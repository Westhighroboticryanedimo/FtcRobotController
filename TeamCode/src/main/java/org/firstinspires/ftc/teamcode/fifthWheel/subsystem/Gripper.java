package org.firstinspires.ftc.teamcode.fifthWheel.subsystem;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Gripper {
    // private Servo grip;
    public ServoEx flipLeft;
    public ServoEx flipRight;
    public ServoEx grip;

    // intake, low, medium, high, cone stack, up
    private static final double LEVELS[] = { -25, 30, 10, -60, -40, -40, -40, -40, 125};
    private static final double OPEN = -20;
    private static final double CLOSE = -86;

    public Gripper(HardwareMap hwMap, String fl, String fr, String gl) {
        flipLeft = new SimpleServo(hwMap, fl, -200, 200);
        flipRight = new SimpleServo(hwMap, fr, -200, 200);
        grip = new SimpleServo(hwMap, gl, -200, 200);

        flipLeft.setInverted(true);
        grip.setInverted(true);
    }

    public void setLevel(int i) {
        switch (i) {
            case -1: // up
                flipLeft.turnToAngle(LEVELS[8]);
                flipRight.turnToAngle(LEVELS[8]);
                break;
            default:
                flipLeft.turnToAngle(LEVELS[i]);
                flipRight.turnToAngle(LEVELS[i]);
                break;
        }
    }

    public void open() {
        grip.turnToAngle(OPEN);
    }

    public void close() {
        grip.turnToAngle(CLOSE);
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
        grip.rotateByAngle(5);
    }

    public void moveClose() {
        grip.rotateByAngle(-5);
    }
}
