package org.firstinspires.ftc.teamcode.thirdWheel.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class LinearSlider extends BaseHardware {
    private DcMotor sliderMotor;
    private int level = 0;

    private final double levelZeroRots     = 0.0;
    private final double levelOneRots      = 3.0/4.0;
    private final double levelTwoRots      = 2.0;
    private final double levelThreeRots    = 13.0/4.0;

    public LinearSlider(LinearOpMode opMode, HardwareMap hwMap) {
        super(opMode);
        init(hwMap);
    }

    public void init(HardwareMap hwMap) {
        sliderMotor = hwMap.get(DcMotor.class, "sliderMotor");
        sliderMotor.setDirection(DcMotor.Direction.FORWARD);
        sliderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderMotor.setMode(DcMotor.RunMode.RUN_WITH_ENCODER);
        sliderMotor.setPower(0);
    }

    // Only moves directly from level 0 to desired level or back to 0
    public int setLevel(int desiredLevel) {
        switch (desiredLevel) {
            case 0:
                // Reverse from the current level
                move(-level);
                break;
            case 1:
                move(levelOneRots);
                break;
            case 2:
                move(levelTwoRots);
                break;
            case 3:
                move(levelThreeRots);
                break;
            default:
                telemetry.addData("bruh", "Invalid level")
                return 1;
                break;
        }
        level = desiredLevel;
        return 0;
    }

    public int getLevel() {
        return level;
    }

    // Moves the slider to the desired level
    private void move(double rotations) {
        sliderMotor.setPower(0.1);
        while ((sliderMotor.getCurrentPosition() / 560.0) != rotations) {}
        sliderMotor.setPower(0);
    }
}
