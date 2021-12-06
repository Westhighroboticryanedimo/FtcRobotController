package org.firstinspires.ftc.teamcode.vampire.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class MiniArm extends BaseHardware {

    // Servos
    private Servo miniClaw;
    private Servo miniArm;
    private static final double THRESHOLD = 0.1;
    private static final double SPEED = 0.03;
    private static final double CLOSE_CLAW = 0;
    private static final double OPEN_CLAW = 0.45;
    private static final double CLOSE_ARM = 0.265;
    private static final double OPEN_ARM = 0.85;

    // Teleop constructor
    public MiniArm(OpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    // Autonomous constructor
    public MiniArm(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    private void setup(HardwareMap hwMap) {

        miniArm = hwMap.get(Servo.class, "miniArm");
        miniArm.setPosition(CLOSE_ARM);
        miniClaw = hwMap.get(Servo.class, "miniClaw");
        miniClaw.setPosition(CLOSE_CLAW);

    }

    public void moveArm(boolean button) {

        print("Miniarm position: ", miniArm.getPosition());
        if (button) {

            if (OPEN_ARM - THRESHOLD < miniArm.getPosition() && miniArm.getPosition() < OPEN_ARM + THRESHOLD) {

                miniClaw.setPosition(CLOSE_CLAW);
                miniArm.setPosition(CLOSE_ARM);

            } else {

                miniClaw.setPosition(OPEN_CLAW);
                miniArm.setPosition(OPEN_ARM);

            }

        }

    }

    public void continuousMoveArm(boolean up, boolean down) {

        if (up) miniArm.setPosition(miniArm.getPosition() - SPEED);
        if (down) miniArm.setPosition(miniArm.getPosition() + SPEED);

    }

    public void moveClaw(boolean button) {

        print("Miniclaw position: ", miniClaw.getPosition());
        if (button) {

            if (OPEN_CLAW - THRESHOLD < miniClaw.getPosition() && miniClaw.getPosition() < OPEN_CLAW + THRESHOLD)
                miniClaw.setPosition(CLOSE_CLAW);
            else miniClaw.setPosition(OPEN_CLAW);

        }

    }

}
