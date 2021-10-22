package org.firstinspires.ftc.teamcode.thirdWheel.hardware;

import java.lang.Math;
import java.lang.Thread;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class LinearSlider extends BaseHardware {
    private DcMotor sliderMotor;
    private int level = 0;
    private int ticks = 0;

    private final int levelZeroTicks     = (0)*560;
    private final int levelOneTicks      = (3/4)*560;
    private final int levelTwoTicks      = (2)*560;
    private final int levelThreeTicks    = (13/4)*560;

    public LinearSlider(LinearOpMode opMode, HardwareMap hwMap) {
        super(opMode);
        init(hwMap);
    }

    public void init(HardwareMap hwMap) {
        sliderMotor = hwMap.get(DcMotor.class, "sliderMotor");
        sliderMotor.setDirection(DcMotor.Direction.FORWARD);
        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotor.setPower(0);
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.RUN_WITH_ENCODER);
    }

    // Move the slider to the position specified, in ticks
    private void move(int desiredTicks) {
        int difference = desiredTicks - ticks;
        // Go forwards/backwards depending on the desired position relative to the current position
        sliderMotor.setPower(0.1 * (difference)/abs(difference));
        // Wait while current difference < difference between old position and desired position
        while (abs(sliderMotor.getCurrentPosition() - ticks) < abs(difference)) { Thread.sleep(1) }
        sliderMotor.setPower(0);
    }

    public int setLevel(int desiredLevel) {
        switch (desiredLevel) {
            case 0:
                move(levelZeroTicks);
                break;
            case 1:
                move(levelOneTicks);
                break;
            case 2:
                move(levelTwoTicks);
                break;
            case 3:
                move(levelThreeTicks);
                break;
            default:
                telemetry.addData("bruh", "Invalid level")
                return 1;
                break;
        }
        ticks = sliderMotor.getCurrentPosition();
        level = desiredLevel;
        return 0;
    }

    public int getLevel() {
        return level;
    }
}
