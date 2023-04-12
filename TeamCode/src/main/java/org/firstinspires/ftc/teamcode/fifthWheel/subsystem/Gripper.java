package org.firstinspires.ftc.teamcode.fifthWheel.subsystem;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Gripper {
    // private Servo grip;
    public ServoEx flipLeft;
    public ServoEx flipRight;
    public ServoEx grip;
    public RevColorSensorV3 color;
    private double thresh = 0.8;
    private int level = 0;
    private int dipDegrees = 50;

    // 0 = red, 1 = blue
    public enum Alliance {
        RED, BLUE
    }

    Alliance alliance = Alliance.RED;

    // intake, low, medium, high, cone stack, up
    // TODO: maybe change up?
    private static final double LEVELS[] = {-65, -60, -90, -180,
            -40, -40, -40, -40,
            -55};
    private static final double OPEN = 30;
    private static final double CLOSE = -5;


    public Gripper(HardwareMap hwMap, String fl, String fr, String gl, Alliance alliance) {
        flipLeft = new SimpleServo(hwMap, fl, -200, 200);
        flipRight = new SimpleServo(hwMap, fr, -200, 200);
        grip = new SimpleServo(hwMap, gl, -200, 200);
        color = hwMap.get(RevColorSensorV3.class, "color");
        this.alliance = alliance;
//        color = new RevColorSensorV3(hwMap, "color");

        flipLeft.setInverted(true);
        grip.setInverted(true);
    }

    public void setLevel(int i) {
        switch (i) {
            case -1: // up
                level = 8;
                flipLeft.turnToAngle(LEVELS[level]);
                flipRight.turnToAngle(LEVELS[level]);
                break;
            default:
                level = i;
                flipLeft.turnToAngle(LEVELS[level]);
                flipRight.turnToAngle(LEVELS[level]);
                break;
        }
    }

    public void dipDown() {
        flipLeft.turnToAngle(LEVELS[level] - dipDegrees);
        flipRight.turnToAngle(LEVELS[level] - dipDegrees);
    }

    public void dipUp() {
        flipLeft.turnToAngle(LEVELS[level]);
        flipRight.turnToAngle(LEVELS[level]);
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

    public boolean in() {
        if (alliance == Alliance.RED) {
            if (color.getDistance(DistanceUnit.INCH) < 4 && color.getNormalizedColors().red > thresh) {
                return true;
            } else {
                return false;
            }
        } else if (alliance == Alliance.BLUE) {
            if (color.getDistance(DistanceUnit.INCH) < 4 && color.getNormalizedColors().blue > thresh) {
                return true;
            } else {
                return false;
            }
        }
        return false;
    }
}
