package org.firstinspires.ftc.teamcode.thirdWheel.hardware.lift;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {
    private DcMotor intakeOne;
    private DcMotor intakeTwo;
    private DistanceSensor distanceSensor;
    private double DIST_THRESH = 2.0;
    private double OUT_THRESH = 5.0;

    public Intake(HardwareMap hwMap) {
        init(hwMap);
    }

    private void init(HardwareMap hwMap) {
        intakeOne = hwMap.get(DcMotor.class, "intakeOne");
        intakeOne.setDirection(DcMotor.Direction.FORWARD);
        intakeOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeOne.setPower(0);
        intakeOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeTwo = hwMap.get(DcMotor.class, "intakeTwo");
        intakeTwo.setDirection(DcMotor.Direction.REVERSE);
        intakeTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeTwo.setPower(0);
        intakeTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        distanceSensor = hwMap.get(DistanceSensor.class, "distanceSensor");
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
        intakeOne.setPower(0.8);
        intakeTwo.setPower(0.8);
    }

    public void out() {
        intakeOne.setPower(-0.8);
        intakeTwo.setPower(-0.8);
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

    public boolean picked() { return distanceSensor.getDistance(DistanceUnit.INCH) < DIST_THRESH; }
    public boolean ejected() { return distanceSensor.getDistance(DistanceUnit.INCH) < OUT_THRESH; }
}
