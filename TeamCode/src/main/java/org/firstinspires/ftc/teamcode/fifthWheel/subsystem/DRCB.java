package org.firstinspires.ftc.teamcode.fifthWheel.subsystem;

import java.lang.Math;
// import java.lang.Thread;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.control.DcMotorUtils;
import org.firstinspires.ftc.teamcode.PIDController;

public class DRCB {
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    private int level = 0;
    private int ticks = 0;

    private static final double LEVELS[] = {0, 80, 100, 160, 320};
    private static final double LIFT_POWER = 0.57;
    private static final double LOWER_POWER = -0.05;

    private PIDController pidArm = new PIDController(1, 0, 0);

    public DRCB(HardwareMap hwMap) {
        init(hwMap);
    }

    private void init(HardwareMap hwMap) {
        pidArm.reset();
        pidArm.setInputRange(0, LEVELS[4]);
        pidArm.setOutputRange(0, LIFT_POWER);
        pidArm.setTolerance(10);

        leftMotor = hwMap.get(DcMotor.class, "leftMotor");
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor = hwMap.get(DcMotor.class, "rightMotor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setPower(0);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    }


    public int getCurrentLeftTicks() {
        return leftMotor.getCurrentPosition();
    }

    public int getCurrentRightTicks() {
        return rightMotor.getCurrentPosition();
    }
}
