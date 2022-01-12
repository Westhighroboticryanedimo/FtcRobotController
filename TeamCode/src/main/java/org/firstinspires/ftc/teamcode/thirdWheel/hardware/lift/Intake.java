package org.firstinspires.ftc.teamcode.thirdWheel.hardware.lift;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intakeOne;
    private DcMotor intakeTwo;

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
        intakeTwo.setPower(1);
    }

    public void out() {
        intakeOne.setPower(-1);
        intakeTwo.setPower(-1);
    }

    public void stop() {
        intakeOne.setPower(0);
        intakeTwo.setPower(0);
    }
}
