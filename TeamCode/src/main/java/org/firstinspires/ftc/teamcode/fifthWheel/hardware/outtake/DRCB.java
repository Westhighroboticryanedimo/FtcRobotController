package org.firstinspires.ftc.teamcode.fifthWheel.hardware.outtake;

import java.lang.Math;
// import java.lang.Thread;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.control.DcMotorUtils;

public class DRCB {
    private DcMotor leftMotor;
    // private DcMotor rightMotor;
    private int level = 0;
    private int ticks = 0;

    private final double LEVELS[] = {0, 1000, 2000, 3000};

    public DRCB(HardwareMap hwMap) {
        init(hwMap);
    }

    private void init(HardwareMap hwMap) {
        leftMotor = hwMap.get(DcMotor.class, "leftMotor");
        // leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setLevel(int l) {
        new Thread() {
            @Override
            public void run() {
                while (!DcMotorUtils.arrived(leftMotor, LEVELS[l], 20)) {
                    DcMotorUtils.moveByTicks(leftMotor, LEVELS[l], 1000, 20);
                }
            }
        }.start();
    }

    public void increaseHeight() {
        new Thread() {
            @Override
            public void run() {
                ticks = DcMotorUtils.getCurrentTicks(leftMotor);
                while (!DcMotorUtils.arrived(leftMotor, ticks + 50, 20)) {
                    DcMotorUtils.moveByTicks(leftMotor, ticks + 50, 50, 20);
                }
            }
        }.start();
    }

    public void decreaseHeight() {
        new Thread() {
            @Override
            public void run() {
                ticks = DcMotorUtils.getCurrentTicks(leftMotor);
                while (!DcMotorUtils.arrived(leftMotor, ticks - 50, 20)) {
                    DcMotorUtils.moveByTicks(leftMotor, ticks - 50, 50, 20);
                }
            }
        }.start();
    }

    public int getCurrentTicks() {
        return DcMotorUtils.getCurrentTicks(leftMotor);
    }
}
