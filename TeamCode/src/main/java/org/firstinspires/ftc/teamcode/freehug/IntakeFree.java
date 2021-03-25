package org.firstinspires.ftc.teamcode.freehug;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class IntakeFree extends BaseHardware {
    private DcMotor intakemotor;
    public double leftPower;
    public double rightPower;

    public IntakeFree(OpMode opMode , HardwareMap hwmap) {
        super(opMode);
        intakemotor = hwmap.get(DcMotor.class, "intake");
        intakemotor.setPower(0);
        intakemotor.setDirection(DcMotor.Direction.FORWARD);

    }

    public void intake(boolean button, boolean button2) {
        if(button) {
            intakemotor.setPower(1);
        }
        else if(button2) {
            intakemotor.setPower(-1);
        } else{
            intakemotor.setPower(0);
        }

    }
}
