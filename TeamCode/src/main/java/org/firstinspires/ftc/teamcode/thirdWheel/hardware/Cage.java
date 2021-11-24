package org.firstinspires.ftc.teamcode.thirdWheel.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class Cage extends BaseHardware {
    public Servo cageServo;
    private int status;

    private final double CAGE_OPEN_POS = 0.83;
    private final double CAGE_CLOSE_POS = 0.45;
    private final double CAGE_DROP_POS = 0.98;

    public Cage(LinearOpMode opMode, HardwareMap hwMap) {
        super(opMode);
        init(hwMap);
    }

    public Cage(OpMode opMode, HardwareMap hwMap) {
        super(opMode);
        init(hwMap);
    }

    private void init(HardwareMap hwMap) {
        cageServo = hwMap.get(Servo.class, "cageServo");
    }

    public void cageOpen() {
        cageServo.setPosition(CAGE_OPEN_POS);
        status = 0;
    }

    public void cageClose() {
        cageServo.setPosition(CAGE_CLOSE_POS);
        status = 1;
    }

    public void cageDrop() {
        cageServo.setPosition(CAGE_DROP_POS);
        status = 2;
    }

    public int getStatus() { return status; }
}
