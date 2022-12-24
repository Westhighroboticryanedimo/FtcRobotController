package org.firstinspires.ftc.teamcode.fifthWheel.subsystem;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Gripper {
    // private Servo grip;
    public ServoEx flipLeft;
    public ServoEx flipRight;
    public ServoEx grip;

    private static final double RIGHT_LEVELS[] = {0.31, 0.54, 0.65, 0.85};
    // intake, ground, low, medium, high, up
    private static final double LEVELS[] = { 17, -20, -55, -90, 90 };
    private static final double OPEN = 125;
    private static final double CLOSE = 30;

    public Gripper(HardwareMap hwMap, String fl, String fr, String g) {
        flipLeft = new SimpleServo(hwMap, fl, -200, 200);
        flipRight = new SimpleServo(hwMap, fr, -200, 200);
        grip = new SimpleServo(hwMap, g, -200, 200);
        flipLeft.setInverted(true);
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
