package org.firstinspires.ftc.teamcode.fifthWheel.hardware.intake;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    // private Servo grip;
    private ServoImplEx flipLeft;
    private ServoImplEx flipRight;

    // private double gripPos = 1;
    // private double flipPos = 0;
    private double rightPos = 0.31;
    private double leftPos = 0.36;

    private static PwmControl.PwmRange SERVO_RANGE = new PwmControl.PwmRange(500, 2500);

    private Boolean closed = false;

    private static final double RIGHT_LEVELS[] = {0.31, 0.54, 0.65, 0.85};
    private static final double LEFT_LEVELS[] = {0.36, 0.32, 0.43, 0.55};

    public Intake(HardwareMap hwMap) {
        init(hwMap);
    }

    private void init(HardwareMap hwMap) {
//        grip = hwMap.get(Servo.class, "grip");
        flipLeft = hwMap.get(ServoImplEx.class, "flipLeft");
        flipRight = hwMap.get(ServoImplEx.class, "flipRight");

        // flipLeft.scaleRange(0, 1);
        flipLeft.setPwmRange(SERVO_RANGE);
        flipRight.setPwmRange(SERVO_RANGE);
        flipRight.setDirection(ServoImplEx.Direction.REVERSE);
        // flipRight.scaleRange(0, 1);
    }

    // public void setLevel(int l) {
    //     flipPos = LEVELS[l];
    //     flipLeft.setPosition(flipPos);
    //     flipRight.setPosition(flipPos);
    // }

    public void open() {
//        rightPos = 0.30;
        if (closed) {
            closed = false;
            rightPos -= 0.18;
            flipRight.setPosition(rightPos);
    //        leftPos = 0.37;
            leftPos += 0.09;
            flipLeft.setPosition(leftPos);
        }
    }

    public void close() {
        if (!closed) {
            closed = true;
            rightPos += 0.18;
            flipRight.setPosition(rightPos);
            leftPos -= 0.09;
            flipLeft.setPosition(leftPos);
        }
    }

    public void lower() {
        rightPos += 0.01;
        leftPos += 0.01;
        flipRight.setPosition(rightPos);
        flipLeft.setPosition(leftPos);
    }

    public void raise() {
        rightPos -= 0.01;
        leftPos -= 0.01;
        flipRight.setPosition(rightPos);
        flipLeft.setPosition(leftPos);
    }

    public void setLevel(int l) {
        rightPos = RIGHT_LEVELS[l];
        leftPos = LEFT_LEVELS[l];
        flipRight.setPosition(rightPos);
        flipLeft.setPosition(leftPos);
    }
//
//    public void one() {
//
//    }
//
//    public void leftIncrease() {
//        leftPos += 0.01;
//        flipLeft.setPosition(leftPos);
//    }
//
//    public void rightIncrease() {
//        rightPos += 0.01;
//        flipRight.setPosition(rightPos);
//    }
//
//    public void leftDecrease() {
//        leftPos -= 0.01;
//        flipLeft.setPosition(leftPos);
//    }
//
//    public void rightDecrease() {
//        rightPos -= 0.01;
//        flipRight.setPosition(rightPos);
//    }

    // public void flipIncrease() {
    //     flipPos += 0.05;
    //     flipLeft.setPosition(flipPos);
    //     flipRight.setPosition(flipPos);
    // }

    // public void flipDecrease() {
    //     flipPos -= 0.05;
    //     flipLeft.setPosition(flipPos);
    //     flipRight.setPosition(flipPos);
    // }

    // public double flipPos() {
    //     return flipPos;
    // }

    public double leftPos() {
        return leftPos;
    }

    public double rightPos() {
        return rightPos;
    }

//    public void open() {
//        gripPos = 1;
//        grip.setPosition(gripPos);
//    }
//
//    public void close() {
//        gripPos = 0.74;
//        grip.setPosition(gripPos);
//    }
//
//    public void gripIncrease() {
//        gripPos -= 0.05;
//        grip.setPosition(gripPos);
//    }
//
//    public void gripDecrease() {
//        gripPos += 0.05;
//        grip.setPosition(gripPos);
//    }
//
//    public double gripPos() {
//        return gripPos;
//    }
}
