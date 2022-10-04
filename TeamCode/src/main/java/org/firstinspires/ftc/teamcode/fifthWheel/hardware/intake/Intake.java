package org.firstinspires.ftc.teamcode.fifthWheel.hardware.intake;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.control.DcMotorUtils;

public class Intake {
    private Servo grip;
    private Servo flipLeft;
    private Servo flipRight;

    public Intake(HardwareMap hwMap) {
        init(hwMap);
    }

    private void init(HardwareMap hwMap) {
        grip = hwMap.get(Servo.class, "claw");
        flipLeft = hwMap.get(Servo.class, "flipLeft");
        flipRight = hwMap.get(Servo.class, "flipRight");

        flipLeft.scaleRange(0.5, 0.9);
        flipRight.setDirection(Servo.Direction.REVERSE);
        flipRight.scaleRange(0.5, 0.9);
    }

    public void lower() {
        flipLeft.setPosition(1);
        flipRight.setPosition(1);
    }

    public void raise() {
        flipLeft.setPosition(0);
        flipRight.setPosition(0);
    }

    public void open() {
        grip.setPosition(0);
    }

    public void close() {
        grip.setPosition(0.5);
    }
}
