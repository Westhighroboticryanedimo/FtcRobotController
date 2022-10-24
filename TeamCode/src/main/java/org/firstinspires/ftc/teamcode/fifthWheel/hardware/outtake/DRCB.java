package org.firstinspires.ftc.teamcode.fifthWheel.hardware.outtake;

import java.lang.Math;
// import java.lang.Thread;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.control.DcMotorUtils;
import org.firstinspires.ftc.teamcode.PIDController;

public class DRCB {
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    // private DcMotor rightMotor;
    private int level = 0;
    private int ticks = 0;

    private static final double LEVELS[] = {0, 60, 120, 180, 360};
    private static final double LIFT_POWER = 0.5;
    private static final double LOWER_POWER = -0.05;

    private PIDController pidArm = new PIDController(0.001, 0, 0.002);

    public DRCB(HardwareMap hwMap) {
        init(hwMap);
    }

    private void init(HardwareMap hwMap) {
        pidArm.reset();
        pidArm.setInputRange(0, LEVELS[4]);
        pidArm.setOutputRange(0, LIFT_POWER);
        pidArm.setTolerance(10);

        leftMotor = hwMap.get(DcMotor.class, "leftMotor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor = hwMap.get(DcMotor.class, "rightMotor");
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setPower(0);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setLevel(int l) {
        new Thread() {
            @Override
            public void run() {
                pidArm.enable();
                pidArm.setSetpoint(LEVELS[l]);
                pidArm.setTolerance(10);
                while (!pidArm.onTarget()) {
                    leftMotor.setPower(pidArm.performPID(leftMotor.getCurrentPosition()));
                    rightMotor.setPower(pidArm.performPID(rightMotor.getCurrentPosition()));
                }
                leftMotor.setPower(Math.sin(leftMotor.getCurrentPosition()/4));
                rightMotor.setPower(Math.sin(rightMotor.getCurrentPosition()/4));
            }
        }.start();
    }

    // public void increaseHeight() {
    //     new Thread() {
    //         @Override
    //         public void run() {
    //             ticks = DcMotorUtils.getCurrentTicks(leftMotor);
    //             while (!DcMotorUtils.arrived(leftMotor, ticks + 500, 100)) {
    //                 DcMotorUtils.moveByTicks(leftMotor, ticks + 500, 100, 100);
    //             }
    //         }
    //     }.start();
    // }

    // public void decreaseHeight() {
    //     new Thread() {
    //         @Override
    //         public void run() {
    //             ticks = DcMotorUtils.getCurrentTicks(leftMotor);
    //             while (!DcMotorUtils.arrived(leftMotor, ticks - 500, 100)) {
    //                 DcMotorUtils.moveByTicks(leftMotor, ticks - 500, 100, 100);
    //             }
    //         }
    //     }.start();
    // }

    public int getCurrentLeftTicks() {
        return leftMotor.getCurrentPosition();
    }

    public int getCurrentRightTicks() {
        return rightMotor.getCurrentPosition();
    }
}
