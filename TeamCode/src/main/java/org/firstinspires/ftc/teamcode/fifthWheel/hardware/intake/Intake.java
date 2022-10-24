package org.firstinspires.ftc.teamcode.fifthWheel.hardware.intake;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private Servo grip;
    private Servo flipLeft;
    private Servo flipRight;

    private double gripPos = 1;
    private double flipPos = 0;

    public Intake(HardwareMap hwMap) {
        init(hwMap);
    }

    private void init(HardwareMap hwMap) {
        grip = hwMap.get(Servo.class, "grip");
        flipLeft = hwMap.get(Servo.class, "flipLeft");
        flipRight = hwMap.get(Servo.class, "flipRight");

        // flipLeft.scaleRange(0, 1);
        flipRight.setDirection(Servo.Direction.REVERSE);
        // flipRight.scaleRange(0, 1);
    }

    public void lower() {
        flipPos = 0.5;
        flipLeft.setPosition(flipPos);
        flipRight.setPosition(flipPos);
    //     flipRight.setPosition(1);
    }

    public void raise() {
        flipPos = 1;
        flipLeft.setPosition(flipPos);
        flipRight.setPosition(flipPos);
    }

    public void flipIncrease() {
        flipPos += 0.05;
        flipLeft.setPosition(flipPos);
        flipRight.setPosition(flipPos);
    }

    public void flipDecrease() {
        flipPos -= 0.05;
        flipLeft.setPosition(flipPos);
        flipRight.setPosition(flipPos);
    }

    public double flipPos() {
        return flipPos;
    }

    public void open() {
        gripPos = 1;
        grip.setPosition(gripPos);
    }

    public void close() {
        gripPos = 0.74;
        grip.setPosition(gripPos);
    }

    public void gripIncrease() {
        gripPos -= 0.05;
        grip.setPosition(gripPos);
    }

    public void gripDecrease() {
        gripPos += 0.05;
        grip.setPosition(gripPos);
    }

    public double gripPos() {
        return gripPos;
    }
}
