package org.firstinspires.ftc.teamcode.reee;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.hardware.Gyro;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

@TeleOp(name="reee teleop", group="reee")

public class reeeTele extends OpMode {

    private DcMotor motor;
    private Servo servo;
    private Controller controller;
    private static double TURN_TICKS = 25.95;

    boolean dir = false;

    byte[] range1Cache;
    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14);
    public static final int RANGE1_REG_START = 0x04;
    public static final int RANGE1_READ_LENGTH = 2;

    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        // servo = hardwareMap.get(Servo.class, "servo");
        // controller = new Controller(gamepad1);
        // RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1 = hardwareMap.get(I2cDevice.class, "range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();
    }

    @Override
    public void loop() {
        if (motor.getCurrentPosition() <= 0) {
            dir = true;
        } else if (motor.getCurrentPosition() >= TURN_TICKS) {
            dir = false;
        }

        if (dir) {
            motor.setPower(0.25);
        } else {
            motor.setPower(-0.25);
        }

        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

        telemetry.addData("ticks:", motor.getCurrentPosition());
        telemetry.addData("ultrasonic:", range1Cache[0] & 0xFF);
        telemetry.addData("optical:", range1Cache[1] & 0xFF);
        telemetry.update();
    }

}
