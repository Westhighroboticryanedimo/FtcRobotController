package org.firstinspires.ftc.teamcode.fifthWheel.subsystem;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Gripper {
    // private Servo grip;
    private ServoEx flipLeft;
    private ServoEx flipRight;
    private ServoEx grip;

    private static final double RIGHT_LEVELS[] = {0.31, 0.54, 0.65, 0.85};
    // intake, ground, low, medium, high, up
    private static final double LEVELS[] = { 0, -5, -20, -60, -90, 90};
    private static final double OPEN = -90;
    private static final double CLOSE = 0;

    public Gripper(HardwareMap hwMap, String fl, String fr, String g) {
        flipLeft = new SimpleServo(hwMap, fl, -90, 90);
        flipRight = new SimpleServo(hwMap, fr, -90, 90);
        grip = new SimpleServo(hwMap, g, -90, 0);
        flipRight.setInverted(true);
    }

    public void setLevel(int i) {
        switch (i) {
            case -1: // up
                flipLeft.turnToAngle(LEVELS[5]);
                flipRight.turnToAngle(LEVELS[5]);
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
}
