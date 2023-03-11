package org.firstinspires.ftc.teamcode.Wingman.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    private DcMotor liftMotor;
    private DcMotor liftMotor2;
    private TouchSensor liftLimit;

    public void liftInit(HardwareMap hardwareMap) {
        liftLimit = hardwareMap.get(TouchSensor.class, "liftLimit");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor2 = hardwareMap.get(DcMotor.class, "liftMotor2");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void raise() {
        if (liftMotor.getCurrentPosition() < -3862) {
            liftMotor.setPower(0);
            liftMotor2.setPower(0);
        } else {
            liftMotor.setPower(-1);
            liftMotor2.setPower(1);
        }
    }

    public void lower() {
        if (liftLimit.isPressed()) {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor.setPower(0);
            liftMotor2.setPower(0);
        } else {
            liftMotor.setPower(0.5);
            liftMotor2.setPower(-0.5);
        }
    }
}
