package org.firstinspires.ftc.teamcode.vampire.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class TapeArm extends BaseHardware {

    // Servos
    private Servo horz;
    private Servo vert;
    private CRServo roll;

    // Constants
    private static final double MOVE_POW = 0.005;
    private static final double ROLL_POW = 1;

    // Teleop constructor
    public TapeArm(OpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    // Autonomous constructor
    public TapeArm(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    private void setup(HardwareMap hwMap) {

        // Set up servos
        horz = hwMap.get(Servo.class, "horz");
        horz.setPosition(0);
        vert = hwMap.get(Servo.class, "vert");
        vert.setPosition(1);
        roll = hwMap.get(CRServo.class, "roll");

    }

    public void horzMove(boolean cw, boolean ccw) {
        print("horz", horz.getPosition());
        if (cw) horz.setPosition(horz.getPosition() - MOVE_POW);
        if (ccw) horz.setPosition(horz.getPosition() + MOVE_POW);

    }

    public void vertMove(boolean up, boolean down) {
        print("vert", vert.getPosition());
        if (up) vert.setPosition(vert.getPosition() + MOVE_POW);
        if (down) vert.setPosition(vert.getPosition() - MOVE_POW);

    }

    public void roll(boolean out, boolean in) {

        if (out) roll.setPower(ROLL_POW);
        else if (in) roll.setPower(-ROLL_POW);
        else roll.setPower(0);

    }

}
