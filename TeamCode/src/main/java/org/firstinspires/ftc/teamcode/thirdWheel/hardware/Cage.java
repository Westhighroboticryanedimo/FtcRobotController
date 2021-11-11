package org.firstinspires.ftc.teamcode.thirdWheel.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class Cage extends BaseHardware {
    private Servo cageServo;

    private final double CAGE_OPEN_POS = 1;
    private final double CAGE_CLOSE_POS = 0;

    public void Cage(LinearOpMode opMode, HardwareMap hwMap) {
        super(opMode);
        init(hwMap);
    }

    public void Cage(OpMode opMode, HardwareMap hwMap) {
        super(opMode);
        init(hwMap);
    }

    private void init(HardwareMap hwMap) {
        cageServo = hwMap.get(Servo.class, "cageServo");
    }

    public void cageOpen() {
        cageServo.setPosition(CAGE_OPEN_POS);
    }
    public void cageClose() {
        cageServo.setPosition(CAGE_CLOSE_POS);
    }
}
