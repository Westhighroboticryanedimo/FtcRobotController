package org.firstinspires.ftc.teamcode.vampire.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class DuckDuckGo extends BaseHardware {

    // Motor and motor power
    private CRServo duckSpin;
    private static final double SPIN_POWER = 0.9;

    // Teleop constructor
    public DuckDuckGo(OpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    // Autonomous constructor
    public DuckDuckGo(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    private void setup(HardwareMap hwMap) {

        // Set up servo
        duckSpin = hwMap.get(CRServo.class, "spin");
        duckSpin.setDirection(DcMotor.Direction.FORWARD);

    }

    public void spin(boolean red, boolean blue) {

        if (red) spinRed();
        else if (blue) spinBlue();
        else stop();

    }

    public void spinRed() {

        duckSpin.setPower(SPIN_POWER);

    }

    public void spinBlue() {

        duckSpin.setPower(-SPIN_POWER);

    }

    public void stop() {

        duckSpin.setPower(0);

    }

}
