package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class BaseHardware {

    // Check if the hardware should add data to telemetry
    protected boolean isDebugMode = false;

    // For teleop
    protected OpMode opMode = null;

    // For autonomous
    protected LinearOpMode linearOpMode = null;

    // Teleop constructor
    public BaseHardware(OpMode opMode) {

        this.opMode = opMode;

    }

    // Autonomous constructor
    public BaseHardware(LinearOpMode opMode) {

        this.linearOpMode = opMode;

    }

    // Set debug mode
    public void debug() {

        isDebugMode = true;

    }

    // Loop here
    public abstract void update();

}
