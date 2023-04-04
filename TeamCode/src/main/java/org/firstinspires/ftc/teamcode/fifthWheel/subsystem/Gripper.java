package org.firstinspires.ftc.teamcode.fifthWheel.subsystem;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Gripper {
    // private Servo grip;
    public ServoEx flipLeft;
    public ServoEx flipRight;
    public ServoEx grip;
    public RevColorSensorV3 color;

    // intake, low, medium, high, cone stack, up
    // TODO: maybe change up?
    private static final double LEVELS[] = { -60, 30, 10, -200, -40, -40, -40, -40, -55};
    private static final double OPEN = 10;
    private static final double CLOSE = -15;

    public Gripper(HardwareMap hwMap, String fl, String fr, String gl) {
        flipLeft = new SimpleServo(hwMap, fl, -200, 200);
        flipRight = new SimpleServo(hwMap, fr, -200, 200);
        grip = new SimpleServo(hwMap, gl, -200, 200);
//        color = new RevColorSensorV3(hwMap, "color");

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
