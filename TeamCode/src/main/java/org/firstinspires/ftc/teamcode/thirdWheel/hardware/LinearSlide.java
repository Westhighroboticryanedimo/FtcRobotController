package org.firstinspires.ftc.teamcode.thirdWheel.hardware;

import java.lang.Math;
import java.lang.Thread;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class LinearSlide extends BaseHardware {
    private DcMotor slideMotor;
    private int level = 0;

    private final int LEV_ZERO_TICKS     = 0;
    private final int LEV_ONE_TICKS      = 840;
    private final int LEV_TWO_TICKS      = 2240;
    private final int LEV_THREE_TICKS    = 3640;

    public LinearSlide(LinearOpMode opMode, HardwareMap hwMap) {
        super(opMode);
        init(hwMap);
    }

    public LinearSlide(OpMode opMode, HardwareMap hwMap) {
        super(opMode);
        init(hwMap);
    }

    private void init(HardwareMap hwMap) {
        slideMotor = hwMap.get(DcMotor.class, "slideMotor");
        slideMotor.setDirection(DcMotor.Direction.REVERSE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setPower(0);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Move the slide to the position specified, in ticks
    private void move(int desiredTicks) {
        int difference = desiredTicks - slideMotor.getCurrentPosition();
        // Go forwards/backwards depending on the desired position relative to the current position
        slideMotor.setPower(1 * difference/Math.abs(difference));
        // Wait until there is no difference between the desired position and the current position
        // Making the threshold 0 might cause the motor to spin infinitely (encoder skips over 0), so 20 for now
        while (Math.abs(desiredTicks - slideMotor.getCurrentPosition()) > 20) {
            // Thread.sleep(1);
        }
        slideMotor.setPower(0);
    }

    public int setLevel(int desiredLevel) {
        switch (desiredLevel) {
            case 0:
                move(LEV_ZERO_TICKS);
                break;
            case 1:
                move(LEV_ONE_TICKS);
                break;
            case 2:
                move(LEV_TWO_TICKS);
                break;
            case 3:
                move(LEV_THREE_TICKS);
                break;
            default:
                // telemetry.addData("bruh", "Invalid level");
                // telemetry.update();
                return 1;
        }
        level = desiredLevel;
        return 0;
    }

    public int getLevel() {
        return level;
    }

    public int getTicks() { return slideMotor.getCurrentPosition(); }
}
