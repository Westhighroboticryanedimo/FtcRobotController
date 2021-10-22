package org.firstinspires.ftc.teamcode.thirdWheel.hardware;

import java.lang.Math;
import java.lang.Thread;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class LinearSlide extends BaseHardware {
    private DcMotor slideMotor;
    private int level = 0;
    private int ticks = 0;

    private final int levelZeroTicks     = (0)*560;
    private final int levelOneTicks      = (3/4)*560;
    private final int levelTwoTicks      = (2)*560;
    private final int levelThreeTicks    = (13/4)*560;

    public LinearSlide(LinearOpMode opMode, HardwareMap hwMap) {
        super(opMode);
        init(hwMap);
    }

    public void init(HardwareMap hwMap) {
        slideMotor = hwMap.get(DcMotor.class, "slideMotor");
        slideMotor.setDirection(DcMotor.Direction.FORWARD);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setPower(0);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITH_ENCODER);
    }

    // Move the slide to the position specified, in ticks
    private void move(int desiredTicks) {
        int difference = desiredTicks - ticks;
        // Go forwards/backwards depending on the desired position relative to the current position
        slideMotor.setPower(0.1 * (difference)/abs(difference));
        // Wait while current difference < difference between old position and desired position
        while (abs(slideMotor.getCurrentPosition() - ticks) < abs(difference)) { Thread.sleep(1) }
        slideMotor.setPower(0);
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
        ticks = slideMotor.getCurrentPosition();
        level = desiredLevel;
        return 0;
    }

    public int getLevel() {
        return level;
    }
}
