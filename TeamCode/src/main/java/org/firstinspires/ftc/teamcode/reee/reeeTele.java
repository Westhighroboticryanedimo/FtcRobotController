package org.firstinspires.ftc.teamcode.reee;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.hardware.Gyro;

@TeleOp(name="reee teleop", group="reee")

public class reeeTele extends OpMode {

    private DcMotor motor;
    private Servo servo;
    private Controller controller;
    private static double TURN_TICKS = 25.95;

    boolean dir= false;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        // servo = hardwareMap.get(Servo.class, "servo");
        // controller = new Controller(gamepad1);
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

        telemetry.addData("ticks:", motor.getCurrentPosition());
        telemetry.update();
    }

}
