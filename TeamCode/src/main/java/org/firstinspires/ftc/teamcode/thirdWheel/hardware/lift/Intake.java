package org.firstinspires.ftc.teamcode.thirdWheel.hardware.lift;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {
    private CRServo intakeOne;
    private CRServo intakeTwo;
    private ColorRangeSensor colorSensor;
    private double DIST_THRESH = 2.0;
    private double OUT_THRESH = 4.0;

    public Intake(HardwareMap hwMap) {
        init(hwMap);
    }

    private void init(HardwareMap hwMap) {
        intakeOne = hwMap.get(CRServo.class, "intakeOne");
        intakeTwo = hwMap.get(CRServo.class, "intakeTwo");
        colorSensor = hwMap.get(ColorRangeSensor.class, "colorSensor");
    }

    public void setState(int state) {
        switch (state) {
            case 0:
                in();
                break;
            case 1:
                out();
                break;
            case 2:
                stop();
                break;
            case -1: // keep the same intake state
                break;
            default:
                // bruh
                break;
        }
    }

    public void in() {
        intakeOne.setPower(1);
        intakeTwo.setPower(-1);
    }

    public void out() {
        intakeOne.setPower(-1);
        intakeTwo.setPower(1);
    }

    public void stop() {
        intakeOne.setPower(0);
        intakeTwo.setPower(0);
    }

    public boolean check() {
        if (picked()) {
            stop();
            return true;
        }
        return false;
    }

    public boolean picked() { return colorSensor.getDistance(DistanceUnit.INCH) < DIST_THRESH; }
    public boolean ejected() { return colorSensor.getDistance(DistanceUnit.INCH) < OUT_THRESH; }
}
