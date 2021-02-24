package org.firstinspires.ftc.teamcode.marinara.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class WobbleDeployer extends BaseHardware {

    private Servo grab;
    private static final double GRAB_POS = 1;
    private static final double RELEASE_POS = 0;

    public WobbleDeployer(OpMode opMode, HardwareMap hwMap) {

        super(opMode);

        initServo(hwMap);

    }

    public WobbleDeployer(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode);

        initServo(hwMap);

    }

    private void initServo(HardwareMap hwMap) {

        grab = hwMap.get(Servo.class, "deployer");
        grab.setPosition(GRAB_POS);

    }

    public void release() {

        grab.setPosition(RELEASE_POS);

    }

}
